# SuperClaude Proxmox Server Build

**Purpose:** Headless VM farm for autonomous Claude Code operations
**Budget:** $2,000
**Date:** 2026-01-15

## Use Case

- MCU/embedded software development (ESP-IDF, etc.)
- Autopilot software development
- Analytical workloads
- VMs created and managed by Claude Code without human intervention

## Existing Equipment (Not Included in Budget)

- Case (suitable for Micro-ATX)
- Power supply
- Multiple 12TB HDDs for bulk storage

---

## Component List

| Component | Model | Price (est.) |
|-----------|-------|--------------|
| CPU | AMD Ryzen 9 7950X | $475 |
| Motherboard | ASRock Rack B650D4U | $299 |
| Memory | 128GB DDR5-4800 ECC UDIMM (4x32GB) | $420 |
| Storage | Samsung 990 Pro 2TB NVMe x2 | $460 |
| Cooling | Noctua NH-D15 | $110 |
| **TOTAL** | | **$1,764** |

**Under budget by $236**

---

## Component Details

### CPU: AMD Ryzen 9 7950X

- 16 cores / 32 threads
- 4.5 GHz base / 5.7 GHz boost
- AM5 socket (LGA 1718)
- 170W TDP (can power-limit to 105W for server use)

**Purchase:**
- [Amazon](https://www.amazon.com/AMD-7950X-32-Thread-Unlocked-Processor/dp/B0BBHD5D8Y)
- [Newegg](https://www.newegg.com/amd-ryzen-9-7950x)

### Motherboard: ASRock Rack B650D4U

- Micro-ATX (9.6" x 9.6")
- **IPMI/BMC included** - remote management without OS
- 4x DDR5 DIMM slots (ECC and non-ECC UDIMM support)
- 1x M.2 PCIe 5.0 x4 + 1x M.2 PCIe 4.0 x4
- 4x SATA 6Gb/s
- Dual 1GbE Intel i210
- Supports Ryzen 9000/8000/7000 up to 170W TDP

**Purchase:**
- [Newegg - $299](https://www.newegg.com/asrock-rack-b650d4u-supports-amd-ryzen-7000-series-processors/p/N82E16813140100)
- [Amazon](https://www.amazon.com/AsRock-B650D4U-Micro-ATX-Motherboard-Processors/dp/B0C7MQMTQW)

**Alternative (10GbE):** ASRock Rack B650D4U-2L2T/BCM (~$420)

### Memory: 128GB DDR5 ECC UDIMM

- Configuration: 4x 32GB DDR5-4800 ECC UDIMM
- Brands: Samsung, Micron, Kingston Server Premier
- ECC for data integrity in server workloads

**Search:** "DDR5 ECC UDIMM 32GB"

### Storage: 2x Samsung 990 Pro 2TB

- PCIe 4.0 x4 NVMe
- 7,450 MB/s read / 6,900 MB/s write
- Ideal for VM storage (fast random I/O)

**Purchase:**
- [Amazon](https://www.amazon.com/SAMSUNG-Internal-Expansion-MZ-V9P2T0B-AM/dp/B0BHJJ9Y77)
- [Best Buy](https://www.bestbuy.com/site/samsung-990-pro-2tb)

### Cooling: Noctua NH-D15

- Dual-tower air cooler
- Handles 7950X at full 170W TDP
- Quiet operation

---

## Why ASRock Rack vs Consumer Boards

| Feature | ASRock Rack B650D4U | Consumer Boards |
|---------|---------------------|-----------------|
| IPMI/BMC | Yes | No |
| ECC Support | Full | Fake/disabled |
| Price | $299 | $400-550 |
| 24/7 Design | Server-grade | Desktop focus |
| Unused Features | None | RGB, audio, etc. |

**IPMI enables:**
- Remote console access (even when OS is dead)
- Remote power cycling
- BIOS access over network
- Hardware health monitoring

---

## Proxmox Setup Notes

### Storage Configuration

- **NVMe:** ZFS mirror for VM disks (fast I/O)
- **HDDs:** ZFS raidz1/raidz2 for bulk storage, ISOs, backups

### VM Templates for Claude Code

1. Ubuntu 22.04/24.04 LTS (general development)
2. ESP-IDF development (pre-installed toolchain)
3. Analytical workloads (Python, Jupyter, etc.)

### BIOS Settings

- Enable IOMMU (AMD-Vi)
- Enable SVM Mode
- Optional: Power limit to 105W for quieter operation

### Automation

- Proxmox API for VM lifecycle management
- Cloud-init for automatic VM configuration
- SSH keys for Claude Code access

---

## Sources

- [ServeTheHome - ASRock Rack AM5 Review](https://www.servethehome.com/asrock-rack-am5d4id-2t-bcm-review-an-awesome-amd-ryzen-server-motherboard-broadcom/)
- [ASRock Rack Products](https://www.asrockrack.com/general/products.asp)
- [Tom's Hardware - Best Motherboards](https://www.tomshardware.com/best-picks/best-motherboards)
