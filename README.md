# An Impulse Engine Written in Rust
[![Build Status](https://travis-ci.com/lizhuohua/impulse-engine-rust-wasm.svg?token=gQ3MGp1DXsVespCpQBDg&branch=master)](https://travis-ci.com/lizhuohua/impulse-engine-rust-wasm)

## [Click here to try it online](https://zhuohua.me/post/an-impulse-engine-in-rust-and-webassembly/)

## Requirements

* Rust nightly
* WebAssembly compilation target: `wasm32-unknown-unknown`

    ```bash
    $ rustup target add wasm32-unknown-unknown
    ```
* cargo-web

    ```bash
    $ cargo install cargo-web
    ```

## Build and Run
```
$ cargo web start --target=wasm32-unknown-unknown
```
Now open `http://localhost:8000` in your browser and try to stack objects!
