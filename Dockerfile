# ---- build stage ----
FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release ca-certificates \
    build-essential cmake pkg-config \
 && curl https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    libgz-sim10-dev \
    libeigen3-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .
RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j"$(nproc)"

# ---- runtime stage ----
FROM ubuntu:24.04 AS runtime
COPY --from=build /src/build/vessel_info /usr/local/bin/vessel_info
# ldd the binary in the build stage to know exactly which .so files to copy in,
# or fall back to apt-installing the matching *-server (non-GUI) runtime package.
ENTRYPOINT ["/usr/local/bin/vessel_info"]