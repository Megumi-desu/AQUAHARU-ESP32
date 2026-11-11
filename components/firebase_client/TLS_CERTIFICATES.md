# Firebase REST TLS trust store

`google_roots.pem` contains four PUBLIC root CA certificates, GTS Root R1, R2,
R3 and R4. These are not credentials or private keys. They were extracted
unchanged from ESP-IDF 6.0.2's `components/mbedtls/esp_crt_bundle/cacrt_all.pem`.
Authoritative Google repository: https://pki.goog/repository/

All four are self-issued roots with expiration 2036-06-22. Keep this file under
version control and update it against Google's official repository when Google
changes its trusted roots. Do not pin a leaf or intermediate certificate, and
do not trust a certificate merely because a failing server supplied it.

The REST client embeds this file using EMBED_TXTFILES and supplies `cert_pem`.
It uses standard mbedTLS chain and hostname verification; `crt_bundle_attach`
is not used by REST. sdkconfig verification settings are not weakened.
The bench does not start SSE; its existing TLS configuration is outside this fix.

The user's serial trace showed successful LCD initialization and Wi-Fi/DHCP,
followed by `No matching trusted root certificate found`, TLS -0x3000 and
REST POST HTTP 0 / ESP_ERR_HTTP_CONNECT. Firebase Auth and RTDB authorization
had not yet been reached. A cross-signed-chain issue in the bundle is a likely
explanation, not a proven identification of the exact chain received by ESP32.

The local SDK already has GTS roots, but cross-signed bundle verification is
disabled. Enabling it selects a callback with separately allocated X.509
subject/name buffers in this SDK; the corresponding free path assumes these
fields belong to the raw certificate buffer. The standard PEM path avoids this
callback. Related upstream report (6.0/6.0.1; inspect installed source rather
than assume a fix is present): https://github.com/espressif/esp-idf/issues/18550

On 7 October 2026, desktop TLS-only handshakes (TLS 1.2 and TLS 1.3) succeeded for the configured
Firebase Auth and RTDB hostnames using ONLY these roots, with certificate and
hostname verification enabled. No HTTP request, credentials, Auth operation,
RTDB read/write, flash, or device operation was performed. This checks root
selection from the desktop network, not ESP32 runtime compatibility or heap
behavior. Those still require serial evidence after the user flashes the build.

The updated ESP-IDF 6.0.2 build passed with `idf.py --no-ccache build`:
binary size 0xe2ca0 bytes, app partition 40% free. Build log:
`build/bench-tls-build.log`. No changes to sdkconfig were required.
