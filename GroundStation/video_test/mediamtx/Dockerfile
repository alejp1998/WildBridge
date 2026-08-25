# WildBridge mediamtx image: adds a static busybox so the container has a
# working wget for Docker healthchecks (the stock image ships only /mediamtx,
# no shell or network tools). The musl build is statically linked, so it runs
# on the scratch-based mediamtx image (the glibc build needs /lib64/ld-linux
# which does not exist there).
FROM bluenviron/mediamtx:latest
COPY --from=busybox:1.36-musl /bin/busybox /bin/busybox
