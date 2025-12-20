# SSL Certificates for AlfieVR

This document explains the SSL certificate setup for the AlfieVR web interface, which is required for WebRTC streaming to mobile devices and VR headsets.

## Why SSL Certificates Are Needed

- **WebRTC requires HTTPS** for camera/microphone access on modern browsers
- **Meta Quest 3 and mobile browsers** are strict about SSL certificate validation
- Self-signed certificates work but must be explicitly trusted on each device

## Current Certificate Details

**Location:** `src/alfie_vr/alfie_vr/cert.pem` and `key.pem`

**Valid For:** 365 days from generation date

**Subject Alternative Names (SANs):**
- `DNS:localhost`
- `IP:127.0.0.1`
- `IP:192.168.5.1` (AlfieNet router)
- `IP:192.168.50.177` (alternative network)

## Symptoms of Expired or Invalid Certificates

### On Desktop (Chrome/Firefox)
- "Your connection is not private" warning
- `NET::ERR_CERT_DATE_INVALID` error
- WebRTC streams fail to connect

### On Android Phone
- Page refuses to load entirely
- "This site can't provide a secure connection" error
- Blank page with no option to proceed

### On Meta Quest 3
- Page shows SSL error with no "Advanced" option
- WebXR/VR mode button doesn't appear
- Streams show as disconnected even after accepting certificates

### In the Application
- Certificate accept buttons appear but clicking them doesn't help
- Status shows "SSL certificate may need to be accepted"
- Console shows `fetch` or WebSocket errors related to SSL

## How to Check Certificate Expiration

```bash
# View certificate details and expiration date
openssl x509 -in src/alfie_vr/alfie_vr/cert.pem -noout -text | grep -A2 "Validity"

# Quick check of expiration date
openssl x509 -in src/alfie_vr/alfie_vr/cert.pem -noout -enddate
```

## How to Regenerate Certificates

### Step 1: Remove Old Certificates

```bash
cd ~/alfiebot_ws/src/alfie_vr/alfie_vr
rm -f cert.pem key.pem
```

### Step 2: Generate New Certificate with SANs

```bash
openssl req -x509 -newkey rsa:2048 \
  -keyout key.pem \
  -out cert.pem \
  -sha256 -days 365 -nodes \
  -subj "/C=US/ST=Test/L=Test/O=AlfieBot/OU=VR/CN=alfie" \
  -addext "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:192.168.5.1,IP:192.168.50.177"
```

**To add additional IP addresses**, append them to the `-addext` line:
```bash
-addext "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:192.168.5.1,IP:192.168.50.177,IP:10.0.0.5"
```

### Step 3: Copy to Bringup (if used)

```bash
cp cert.pem ~/alfiebot_ws/src/alfie_bringup/certs/
cp key.pem ~/alfiebot_ws/src/alfie_bringup/certs/
```

### Step 4: Verify the New Certificate

```bash
openssl x509 -in cert.pem -noout -text | grep -A2 "Subject Alternative Name"
```

Expected output:
```
X509v3 Subject Alternative Name: 
    DNS:localhost, IP Address:127.0.0.1, IP Address:192.168.5.1, IP Address:192.168.50.177
```

### Step 5: Restart the VR Server

Restart the `alfie_vr` node to use the new certificates.

## Accepting Certificates on Devices

After regenerating certificates, each device must accept the new certificate.

### Desktop Browser (Chrome/Firefox)
1. Navigate to `https://192.168.5.1:8443` (or your IP/port)
2. Click "Advanced" → "Proceed to site (unsafe)"
3. Repeat for each port used (web server, stereo signaling, RGB signaling)

### Android Phone
1. Open Chrome and navigate to `https://192.168.5.1:8443`
2. Tap "Advanced" → "Proceed to site"
3. Repeat for signaling ports
4. **Alternative:** Install certificate in Settings → Security → Install certificates

### Meta Quest 3
1. Open the Quest Browser
2. Navigate to `https://192.168.5.1:8443`
3. Accept the security warning
4. Navigate to each signaling port URL and accept those certificates too
5. Return to the main VR page and reload

## Troubleshooting

### "Certificate already accepted but still not working"

Each HTTPS port needs its certificate accepted separately:
- Web UI port (e.g., 8443)
- Stereo signaling port (e.g., 8444)
- RGB signaling port (e.g., 8445)

### "Works on PC but not on mobile"

Mobile browsers are stricter. Ensure:
1. The certificate has proper SANs (not just CN=localhost)
2. You're accessing via an IP address listed in the SANs
3. The certificate hasn't expired

### "Quest 3 shows blank page"

1. Clear the Quest browser cache
2. Restart the Quest browser
3. Try accessing each port URL directly first to accept certs

## Certificate Renewal Reminder

**Current certificate expires:** Check with the command above

**Recommended:** Set a calendar reminder to regenerate certificates before expiration (e.g., every 11 months).

## Future Improvements

Consider using:
- **mkcert** - Creates locally-trusted certificates automatically
- **Let's Encrypt** - Free trusted certificates (requires domain name)
- **Tailscale** - Automatic HTTPS for devices on your tailnet
