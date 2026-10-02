//! `zenoh-dimos-codecs-example`: lists the codecs, then starts a zenoh-web server with all of them on 127.0.0.1
//! (isolated zenoh session: no scouting, no listeners) and shuts it down.

use anyhow::Result;
use zenoh_web::{Server, zenoh};

#[tokio::main]
async fn main() -> Result<()> {
    if std::env::args().nth(1).is_some_and(|argument| argument == "-h" || argument == "--help") {
        println!("usage: zenoh-dimos-codecs-example  (lists the codecs and starts/stops a loopback server with them)");
        return Ok(());
    }
    let codecs = zenoh_dimos_codecs::all();
    println!("codecs: {:?}", codecs.iter().map(|codec| codec.name().to_owned()).collect::<Vec<_>>());
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false").map_err(anyhow::Error::msg)?;
    config.insert_json5("listen/endpoints", "[]").map_err(anyhow::Error::msg)?;
    let mut builder = Server::builder().zenoh_config(config);
    for codec in codecs {
        builder = builder.shared_codec(codec);
    }
    let running = builder.build().await?.bind("127.0.0.1:0").await?;
    println!("serving on http://{}", running.local_addr());
    running.shutdown().await?;
    println!("ok");
    Ok(())
}
