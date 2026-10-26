---
name: ros2-networking
description: "Use when diagnosing ROS 2 networking: RMW, DDS, Fast DDS, Cyclone DDS, Connext, discovery, QoS, multicast, domains, Linux Windows macOS firewalls, security, and version-validated rmw_zenoh."
user-invocable: true
---

# ROS 2 Networking and Middleware

## Required preflight
- Read [common platform and safety guidance](../ros2-development/SKILL.md) before executing tasks; stop if unavailable.
- Detect OS, shell, architecture, ROS distribution/version, active environment, and whether execution is local, remote, or containerized.
- Locate compatible ROS and network diagnostic binaries; use discovered shell-compatible setup scripts only.
- Do not assume paths, run Unix shims on Windows, or silently switch shells, containers, or networks.
- Confirm the approved simulation versus hardware target and authorized network/domain before discovery probes.
- Do not operate a live robot, arm, or drone without explicit approval; never publish test commands onto an unknown graph.

## Establish the baseline
1. Record effective RMW implementation, package versions, domain ID, discovery restrictions, namespaces, and security mode on each endpoint.
2. Check advertised topic/service types and message-definition compatibility before blaming the network.
3. Compare interface addresses, routes, VPNs, container boundaries, and IPv4/IPv6 selection using OS-appropriate diagnostics.
4. Inspect participant logs and effective configuration; sanitize credentials and sensitive network details in reports.
5. Separate discovery failure from discovered endpoints that cannot exchange data.

## DDS investigation
1. Verify the installed Fast DDS, Cyclone DDS, or Connext RMW and its support for the target ROS/platform combination.
2. Consult documentation matching the installed vendor version before writing XML profiles or choosing environment variables.
3. Check interface binding, multicast availability, discovery peers/servers, and transport settings on both sides.
4. Validate discovery-server/static-peer features against the selected vendor; configuration is not interchangeable across DDS vendors.
5. On Linux, inspect routing, firewall rules, and container namespaces; on Windows, inspect network profile and application firewall rules.
6. On macOS, check firewall permissions and VM/container networking; do not assume Linux host-network behavior is available.
7. Test multicast only on an approved network with compatible sender/receiver tools; a successful ping proves neither multicast nor DDS connectivity.
8. Derive firewall port requirements from actual vendor/domain/participant configuration; do not broadly disable firewalls.
9. Match intended domain IDs and discovery scope; a domain ID isolates discovery logically, not as a security boundary.

## QoS and graph edge cases
- Compare offered publisher and requested subscriber reliability, durability, history/depth, deadline, and liveliness.
- A reliable subscriber cannot match a best-effort publisher; adjust according to data-loss requirements, not guesswork.
- Check transient-local expectations for late joiners and bounded histories for large images or point clouds.
- Inspect incompatible-QoS events and endpoint details; discovered topics alone do not prove compatible delivery.
- Check CLI/daemon middleware and domain context when graph output disagrees with application behavior.
- Restart a stale local daemon only with an understood scope; it is not a substitute for correcting participant configuration.

## Security and non-DDS middleware
1. Validate DDS Security/SROS 2 enclaves, governance, permissions, certificate validity, and vendor support without printing private keys.
2. Preserve enforced security; fix trust or policy mismatches instead of silently turning authentication off.
3. Treat `rmw_zenoh` as a distinct non-DDS transport, not a DDS XML profile or multicast tuning variant.
4. Verify the exact ROS distribution, rmw_zenoh package/version, platform support, and documented configuration schema.
5. Inspect required routers, endpoints, discovery/session behavior, and security options for that installed version.
6. Do not assume transparent communication between DDS and Zenoh participants without a supported, explicitly configured bridge.

## Validation
- Reproduce with an approved minimal talker/listener or recorded-data test, first locally and then across authorized hosts.
- Measure delivery, latency, loss, reconnect behavior, and late-join behavior with representative message sizes and QoS.
- Change one variable at a time and preserve a rollback copy of configuration; never expose keys in captures.
- Report the host/RMW/version matrix, validated configuration, remaining network constraints, and untested combinations.
