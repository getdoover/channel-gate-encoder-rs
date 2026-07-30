# syntax=docker/dockerfile:1
#
# channel-gate-encoder -- multi-arch static-musl device app image.
#
# Follows the pattern established by doover-rs's own Dockerfile: ONE native
# rustc on $BUILDPLATFORM cross-compiles every architecture via cargo-zigbuild
# (zig supplies the cross C compiler/linker). No QEMU, no per-arch base image --
# only the Rust target triple changes. This matters here specifically because
# the target Doovit has NO rustc installed and 1.8 GB of RAM; a native cargo
# build on the device is not a realistic option.
#
# Build the arm64 image the Doovits run:
#
#   docker buildx build --platform linux/arm64 \
#     -t ghcr.io/getdoover/channel-gate-encoder-rs:main --load .
#
# Or both architectures, as doover_config.json's build_args asks for:
#
#   docker buildx build --platform linux/amd64,linux/arm64 \
#     -t ghcr.io/getdoover/channel-gate-encoder-rs:main --push .
#
# Export just the binary (for `scp`-and-run debugging on a device):
#
#   docker buildx build --platform linux/arm64 --target bin \
#     --output type=local,dest=./dist .
#
# A static musl binary shares only the host KERNEL, so one arm64 build runs on
# the CM4 Doovits (Debian 12) and on far older userlands alike -- and the final
# image is FROM scratch, so there is no base image to keep patched.

ARG ZIG_VERSION=0.13.0

FROM --platform=$BUILDPLATFORM rust:1-bookworm AS builder
ARG ZIG_VERSION
ARG TARGETPLATFORM
ARG TARGETARCH
ARG TARGETVARIANT

RUN apt-get update && apt-get install -y --no-install-recommends \
        xz-utils curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/*

# NB: no system protoc -- doover-proto/build.rs falls back to its vendored
# protoc when PROTOC is unset (a glibc binary, which runs natively on this
# builder even when the TARGET is musl/arm64).
RUN set -eux; \
    case "$(uname -m)" in \
        aarch64) ZARCH=aarch64 ;; \
        x86_64)  ZARCH=x86_64  ;; \
        *) echo "unsupported build arch $(uname -m)" >&2; exit 1 ;; \
    esac; \
    curl -fsSL "https://ziglang.org/download/${ZIG_VERSION}/zig-linux-${ZARCH}-${ZIG_VERSION}.tar.xz" -o /tmp/zig.tar.xz; \
    mkdir -p /opt/zig; tar -xJf /tmp/zig.tar.xz -C /opt/zig --strip-components=1; \
    ln -s /opt/zig/zig /usr/local/bin/zig; \
    zig version
RUN cargo install cargo-zigbuild --locked

RUN set -eux; \
    case "$TARGETPLATFORM" in \
        linux/amd64)  TRIPLE=x86_64-unknown-linux-musl      ;; \
        linux/arm64)  TRIPLE=aarch64-unknown-linux-musl     ;; \
        linux/arm/v7) TRIPLE=armv7-unknown-linux-musleabihf ;; \
        *) echo "unsupported target platform: $TARGETPLATFORM" >&2; exit 1 ;; \
    esac; \
    echo "$TRIPLE" > /tmp/triple; \
    rustup target add "$TRIPLE"

WORKDIR /build
COPY Cargo.toml Cargo.lock ./
COPY src/ ./src/

# Per-arch cache ids so concurrent multi-platform builds don't race on the
# crate registry or the target dir.
RUN --mount=type=cache,target=/usr/local/cargo/registry,id=cge-registry-${TARGETARCH}${TARGETVARIANT} \
    --mount=type=cache,target=/build/target,id=cge-target-${TARGETARCH}${TARGETVARIANT} \
    set -eux; \
    TRIPLE="$(cat /tmp/triple)"; \
    cargo zigbuild --release --locked --target "$TRIPLE" --bin channel-gate-encoder; \
    cp "target/${TRIPLE}/release/channel-gate-encoder" /channel-gate-encoder; \
    strip /channel-gate-encoder || true; \
    ls -l /channel-gate-encoder

# Binary-only export stage: `--target bin --output type=local` drops the binary
# on the host with no image built.
FROM scratch AS bin
COPY --from=builder /channel-gate-encoder /channel-gate-encoder

# The deployable app image. A static binary needs nothing else -- not even libc.
FROM scratch AS final_image
LABEL com.doover.app="true"
LABEL com.doover.managed="true"
COPY --from=builder /channel-gate-encoder /channel-gate-encoder
ENV HEALTHCHECK_PORT=49200
# The Python app's HEALTHCHECK shells out to curl. There is no curl (or shell)
# in a scratch image, so the binary probes its own endpoint instead -- same
# semantics as `curl -f 127.0.0.1:$HEALTHCHECK_PORT`, zero extra bytes.
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
    CMD ["/channel-gate-encoder", "healthcheck"]
ENTRYPOINT ["/channel-gate-encoder"]
