# Clean-room packager (E2 exit proof): builds the embedding ABI from a
# bare source tree — no caches, no host toolchains — installs the
# `packaging/README.md` layout, then builds and runs both minimal hosts
# against the installed files.
#
#   docker build -f packaging/Dockerfile -t vioarm-packaging .
# (The build itself is the test: any failure fails the image build.)

FROM rust:bookworm
# Note: current stable, not the 1.83 MSRV floor — the Cargo.lock pins
# dependency manifests newer cargo features than 1.83-era cargo parses.
# MSRV itself is pinned by `cargo +1.83 check` in CI, not here.
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake python3 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY vioarm-core vioarm-core
COPY vioarm-cli vioarm-cli
COPY vioarm-svd vioarm-svd
COPY vioarm-gdb vioarm-gdb
COPY vioarm-qemu vioarm-qemu
COPY vioarm-jit-call vioarm-jit-call
COPY vioarm-cosim vioarm-cosim
COPY bindings bindings
COPY packaging packaging
RUN cargo build --release -p vioarm-cosim
RUN mkdir -p /opt/vioarm/include /opt/vioarm/lib && \
    cp target/release/libvioarm_cosim.so target/release/libvioarm_cosim.a /opt/vioarm/lib/ && \
    ln -sf libvioarm_cosim.so /opt/vioarm/lib/libvioarm_cosim.so.1 && \
    cp vioarm-cosim/include/vioarm_cosim.h /opt/vioarm/include/ && \
    objdump -p /opt/vioarm/lib/libvioarm_cosim.so | grep -q "SONAME.*libvioarm_cosim.so.1"
RUN gcc packaging/examples/host_minimal.c -I/opt/vioarm/include \
        /opt/vioarm/lib/libvioarm_cosim.so -o /tmp/host_minimal && \
    LD_LIBRARY_PATH=/opt/vioarm/lib /tmp/host_minimal
RUN VIOARM_LIB=/opt/vioarm/lib/libvioarm_cosim.so \
    PYTHONPATH=/src/bindings/python \
    python3 packaging/examples/host_minimal.py
RUN VIOARM_LIB=/opt/vioarm/lib/libvioarm_cosim.so \
    PYTHONPATH=/src/bindings/python \
    python3 packaging/examples/rate_limited.py --scale 0.01 --quanta 5 --quantum-us 100
