{
    description = "zenoh-dimos-codecs: zenoh-web codecs for ROS 2 and dimos messages (crate2nix builds, native and aarch64 Linux)";

    # zenoh-web's lib.crossRust: its nixpkgs / rust-overlay pins, so crates are shared with the other zenoh-web flakes
    inputs.zenoh-web.url = "github:jeff-hykin/zenoh-web";

    outputs = { self, zenoh-web }: {
        # the library through nix_smoke_test/: a loopback server with every codec
        packages = zenoh-web.lib.eachSystem (system:
            let built = zenoh-web.lib.crossRustPackages { name = "zenoh-dimos-codecs-example"; inherit system; cargoNix = ./nix_smoke_test/Cargo.nix; };
            in built // { default = built.zenoh-dimos-codecs-example; });
        devShells = zenoh-web.devShells;
    };
}
