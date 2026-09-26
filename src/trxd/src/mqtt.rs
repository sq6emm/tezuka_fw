//! MQTT: decodes and state out, commands in.
//!
//! * `<prefix>/online`          retained `true`/`false` (last will)
//! * `<prefix>/state`           retained JSON snapshot, on change
//! * `<prefix>/decode/<mode>`   one JSON [`crate::decode::Decode`] per decode
//! * `<prefix>/cmd/<name>`      subscribed; payload is the value
//!
//! A broker that is down never blocks the radio: publishes are queued in
//! rumqttc's bounded request channel and dropped when it is full.

use std::time::Duration;

use crossbeam_channel::{Receiver, unbounded};
use rumqttc::{Client, Event, LastWill, MqttOptions, Packet, QoS};
use serde::Serialize;
use tracing::{debug, info, warn};

use crate::config::{Config, MqttConfig};

pub struct Mqtt {
    client: Option<Client>,
    prefix: String,
    cmds: Receiver<(String, String)>,
}

impl Mqtt {
    /// A handle that publishes nowhere, for `mqtt.enabled = false` and tests.
    pub fn disabled() -> Self {
        Mqtt { client: None, prefix: String::new(), cmds: unbounded().1 }
    }

    pub fn start(cfg: &Config) -> Self {
        let m: &MqttConfig = &cfg.mqtt;
        if !m.enabled {
            return Self::disabled();
        }
        let prefix = cfg.topic_prefix();
        let id = if m.client_id.is_empty() { format!("trxd-{}", Config::hostname()) } else { m.client_id.clone() };
        let mut opts = MqttOptions::new(id, &m.host, m.port);
        opts.set_keep_alive(Duration::from_secs(30));
        opts.set_last_will(LastWill::new(format!("{prefix}/online"), "false", QoS::AtLeastOnce, true));
        if !m.username.is_empty() {
            opts.set_credentials(&m.username, &m.password);
        }
        let (client, mut connection) = Client::new(opts, 256);
        let (cmd_tx, cmds) = unbounded();
        let sub_client = client.clone();
        let cmd_prefix = format!("{prefix}/cmd/");
        let online = format!("{prefix}/online");
        std::thread::Builder::new()
            .name("mqtt".into())
            .spawn(move || {
                for ev in connection.iter() {
                    match ev {
                        Ok(Event::Incoming(Packet::ConnAck(_))) => {
                            info!("MQTT connected");
                            let _ = sub_client.try_subscribe(format!("{cmd_prefix}#"), QoS::AtLeastOnce);
                            let _ = sub_client.try_publish(&online, QoS::AtLeastOnce, true, "true");
                        }
                        Ok(Event::Incoming(Packet::Publish(p))) => {
                            if let Some(name) = p.topic.strip_prefix(&cmd_prefix) {
                                let payload = String::from_utf8_lossy(&p.payload).trim().to_string();
                                debug!(cmd = name, %payload, "MQTT command");
                                if cmd_tx.send((name.to_string(), payload)).is_err() {
                                    break;
                                }
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            warn!("MQTT: {e}");
                            std::thread::sleep(Duration::from_secs(5));
                        }
                    }
                }
            })
            .expect("spawn mqtt thread");
        info!(broker = %format!("{}:{}", m.host, m.port), %prefix, "MQTT");
        Mqtt { client: Some(client), prefix, cmds }
    }

    /// A second handle that publishes on the same connection (commands stay
    /// with the original).
    pub fn publisher(&self) -> Mqtt {
        Mqtt { client: self.client.clone(), prefix: self.prefix.clone(), cmds: unbounded().1 }
    }

    pub fn publish_json<T: Serialize>(&self, sub: &str, value: &T, retain: bool) {
        let Some(c) = &self.client else { return };
        match serde_json::to_vec(value) {
            Ok(body) => {
                if c.try_publish(format!("{}/{sub}", self.prefix), QoS::AtMostOnce, retain, body).is_err() {
                    debug!("MQTT queue full, dropped {sub}");
                }
            }
            Err(e) => warn!("MQTT encode {sub}: {e}"),
        }
    }

    pub fn publish_decode(&self, d: &crate::decode::Decode) {
        let mode = d.mode.to_ascii_lowercase();
        self.publish_json(&format!("decode/{mode}"), d, false);
    }

    /// Commands received since the last call: `(name, payload)`.
    pub fn poll_cmds(&self) -> Vec<(String, String)> {
        self.cmds.try_iter().collect()
    }
}
