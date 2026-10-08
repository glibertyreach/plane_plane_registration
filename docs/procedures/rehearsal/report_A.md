# Registration report

Manifest: /tmp/claude-0/-home-user/fd747cf1-d9f9-5dd3-84db-c067a012fbe6/scratchpad/rehearsal/captures_A/manifest.csv

Rough transform for the first pass: /tmp/claude-0/-home-user/fd747cf1-d9f9-5dd3-84db-c067a012fbe6/scratchpad/rehearsal/boot.json.
Outlier rejection rounds: 0.

## Model: rigid

Verdict: NOT ACCEPTED. Status ResidualsExceedThreshold: registration solved, but a residual exceeds its acceptance threshold. Final pass: pass2; 235 of 235 poses used.

### Transform (sensor to base)

```
  1.000000  -0.000064  -0.000017 799.971997
 -0.000064  -1.000000  -0.000002  -0.010700
 -0.000017   0.000002  -1.000000 1200.015742
  0.000000   0.000000   0.000000   1.000000
```

Rotation vector 179.9999, -0.0057, -0.0015 deg; translation 799.9720, -0.0107, 1200.0157 mm; scale 1.000000.

### Residuals against the acceptance thresholds

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0286 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.1659 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1587 | 0.5000 | within the limit |
| maximum offset residual (mm) | 1.2699 | 1.0000 | EXCEEDS the limit |

### Spreads of the pose set

| quantity | value | minimum | result |
|---|---|---|---|
| normal spread | 0.2850 | 0.0500 | at or above the minimum |

### Passes

| pass | status (candidate regions from) | poses used | RMS normal (deg) | RMS offset (mm) |
|---|---|---|---|---|
| pass1 | ResidualsExceedThreshold (sensor_in_base) | 235/235 | 0.0286 | 0.1591 |
| pass2 | ResidualsExceedThreshold (pass1_transform) | 235/235 | 0.0286 | 0.1587 |

### Segmentation

Segmentation methods: predicted_region 235.
Mask pixels per pose: minimum 490, median 10971, maximum 35048.
Poses whose segmentation failed: none.
Poses rejected as outliers: none.

### Residual of every pose

| pose | method | pixels | plane RMS (mm) | normal (deg) | offset (mm) | used |
|---|---|---|---|---|---|---|
| b_z0550_t00_a000_x0y0 | predicted_region | 35047 | 0.0170 | 0.0238 | 0.0944 | yes |
| b_z0550_t15_a000_x0y0 | predicted_region | 31629 | 0.0169 | 0.0286 | -0.0936 | yes |
| b_z0550_t15_a090_x0y0 | predicted_region | 31356 | 0.0171 | 0.0225 | -0.1521 | yes |
| b_z0550_t15_a180_x0y0 | predicted_region | 31369 | 0.0166 | 0.0205 | 0.0658 | yes |
| b_z0550_t15_a270_x0y0 | predicted_region | 30642 | 0.0167 | 0.0252 | 0.0299 | yes |
| b_z0550_t30_a180_x0y0 | predicted_region | 12492 | 0.0164 | 0.0117 | -0.0203 | yes |
| b_z0550_t30_a270_x0y0 | predicted_region | 13141 | 0.0163 | 0.0294 | 0.0974 | yes |
| b_z0550_t00_a000_x1y0 | predicted_region | 33850 | 0.0160 | 0.0240 | -0.0545 | yes |
| b_z0550_t15_a000_x1y0 | predicted_region | 29001 | 0.0168 | 0.0300 | -0.0830 | yes |
| b_z0550_t15_a090_x1y0 | predicted_region | 29913 | 0.0161 | 0.0238 | 0.1518 | yes |
| b_z0550_t15_a180_x1y0 | predicted_region | 28975 | 0.0165 | 0.0214 | -0.0194 | yes |
| b_z0550_t15_a270_x1y0 | predicted_region | 31109 | 0.0165 | 0.0278 | 0.0306 | yes |
| b_z0550_t30_a000_x1y0 | predicted_region | 14284 | 0.0160 | 0.0293 | 0.1396 | yes |
| b_z0550_t30_a090_x1y0 | predicted_region | 12905 | 0.0154 | 0.0250 | 0.2132 | yes |
| b_z0550_t30_a180_x1y0 | predicted_region | 12591 | 0.0166 | 0.0145 | 0.0127 | yes |
| b_z0550_t30_a270_x1y0 | predicted_region | 13909 | 0.0165 | 0.0282 | 0.1078 | yes |
| b_z0550_t00_a000_x2y0 | predicted_region | 34822 | 0.0162 | 0.0273 | -0.0313 | yes |
| b_z0550_t15_a000_x2y0 | predicted_region | 30459 | 0.0168 | 0.0299 | -0.0682 | yes |
| b_z0550_t15_a090_x2y0 | predicted_region | 32654 | 0.0168 | 0.0247 | -0.1154 | yes |
| b_z0550_t15_a180_x2y0 | predicted_region | 31423 | 0.0165 | 0.0223 | 0.0354 | yes |
| b_z0550_t15_a270_x2y0 | predicted_region | 31088 | 0.0166 | 0.0279 | -0.0009 | yes |
| b_z0550_t30_a090_x2y0 | predicted_region | 15088 | 0.0166 | 0.0246 | 0.2125 | yes |
| b_z0550_t30_a180_x2y0 | predicted_region | 11960 | 0.0167 | 0.0193 | 0.0482 | yes |
| b_z0550_t00_a000_x2y1 | predicted_region | 34271 | 0.0164 | 0.0249 | -0.0286 | yes |
| b_z0550_t15_a000_x2y1 | predicted_region | 29600 | 0.0167 | 0.0275 | -0.0684 | yes |
| b_z0550_t15_a090_x2y1 | predicted_region | 29447 | 0.0164 | 0.0223 | 0.1326 | yes |
| b_z0550_t15_a180_x2y1 | predicted_region | 31150 | 0.0172 | 0.0213 | 0.0250 | yes |
| b_z0550_t15_a270_x2y1 | predicted_region | 32249 | 0.0162 | 0.0272 | 0.0494 | yes |
| b_z0550_t30_a000_x2y1 | predicted_region | 12586 | 0.0165 | 0.0305 | -0.0962 | yes |
| b_z0550_t30_a090_x2y1 | predicted_region | 13306 | 0.0168 | 0.0179 | -0.1681 | yes |
| b_z0550_t30_a180_x2y1 | predicted_region | 12349 | 0.0162 | 0.0180 | -0.0338 | yes |
| b_z0550_t30_a270_x2y1 | predicted_region | 14797 | 0.0156 | 0.0288 | -0.0721 | yes |
| b_z0550_t00_a000_x1y1 | predicted_region | 33227 | 0.0157 | 0.0219 | -0.0508 | yes |
| b_z0550_t15_a000_x1y1 | predicted_region | 31675 | 0.0163 | 0.0268 | -0.0827 | yes |
| b_z0550_t15_a090_x1y1 | predicted_region | 27607 | 0.0163 | 0.0223 | 0.1397 | yes |
| b_z0550_t15_a180_x1y1 | predicted_region | 31655 | 0.0162 | 0.0183 | 0.0445 | yes |
| b_z0550_t15_a270_x1y1 | predicted_region | 29993 | 0.0166 | 0.0260 | 0.0160 | yes |
| b_z0550_t30_a000_x1y1 | predicted_region | 5019 | 0.0154 | 0.0315 | -0.1183 | yes |
| b_z0550_t30_a090_x1y1 | predicted_region | 14964 | 0.0157 | 0.0163 | -0.1787 | yes |
| b_z0550_t30_a180_x1y1 | predicted_region | 11547 | 0.0160 | 0.0121 | 0.0161 | yes |
| b_z0550_t30_a270_x1y1 | predicted_region | 13344 | 0.0149 | 0.0255 | -0.0381 | yes |
| b_z0550_t00_a000_x0y1 | predicted_region | 34474 | 0.0165 | 0.0217 | 0.0887 | yes |
| b_z0550_t15_a000_x0y1 | predicted_region | 29705 | 0.0163 | 0.0252 | 0.1095 | yes |
| b_z0550_t15_a090_x0y1 | predicted_region | 32423 | 0.0161 | 0.0200 | -0.1435 | yes |
| b_z0550_t15_a180_x0y1 | predicted_region | 31533 | 0.0168 | 0.0161 | 0.0638 | yes |
| b_z0550_t15_a270_x0y1 | predicted_region | 30265 | 0.0165 | 0.0217 | 0.0009 | yes |
| b_z0550_t30_a000_x0y1 | predicted_region | 13522 | 0.0163 | 0.0277 | 0.1260 | yes |
| b_z0550_t30_a090_x0y1 | predicted_region | 15909 | 0.0156 | 0.0213 | -0.2037 | yes |
| b_z0550_t30_a180_x0y1 | predicted_region | 14949 | 0.0170 | 0.0078 | -0.0157 | yes |
| b_z0550_t30_a270_x0y1 | predicted_region | 13100 | 0.0165 | 0.0242 | 0.0758 | yes |
| b_z0550_t00_a000_x0y2 | predicted_region | 35028 | 0.0175 | 0.0148 | -0.0669 | yes |
| b_z0550_t15_a000_x0y2 | predicted_region | 33016 | 0.0168 | 0.0235 | 0.1063 | yes |
| b_z0550_t15_a090_x0y2 | predicted_region | 33467 | 0.0166 | 0.0181 | 0.1247 | yes |
| b_z0550_t15_a180_x0y2 | predicted_region | 30380 | 0.0172 | 0.0127 | 0.0507 | yes |
| b_z0550_t15_a270_x0y2 | predicted_region | 30477 | 0.0163 | 0.0175 | -0.0048 | yes |
| b_z0550_t30_a000_x0y2 | predicted_region | 13482 | 0.0170 | 0.0270 | -0.0757 | yes |
| b_z0550_t30_a270_x0y2 | predicted_region | 13363 | 0.0162 | 0.0211 | 0.0599 | yes |
| b_z0550_t00_a000_x1y2 | predicted_region | 34063 | 0.0160 | 0.0175 | -0.0470 | yes |
| b_z0550_t15_a000_x1y2 | predicted_region | 30996 | 0.0166 | 0.0250 | 0.1107 | yes |
| b_z0550_t15_a090_x1y2 | predicted_region | 28352 | 0.0164 | 0.0178 | 0.1216 | yes |
| b_z0550_t15_a180_x1y2 | predicted_region | 31186 | 0.0166 | 0.0128 | -0.0115 | yes |
| b_z0550_t15_a270_x1y2 | predicted_region | 31404 | 0.0159 | 0.0232 | 0.0181 | yes |
| b_z0550_t30_a000_x1y2 | predicted_region | 12555 | 0.0165 | 0.0271 | 0.1410 | yes |
| b_z0550_t30_a090_x1y2 | predicted_region | 14144 | 0.0163 | 0.0138 | -0.1589 | yes |
| b_z0550_t30_a180_x1y2 | predicted_region | 13559 | 0.0164 | 0.0114 | 0.0274 | yes |
| b_z0550_t30_a270_x1y2 | predicted_region | 13287 | 0.0154 | 0.0273 | 0.0919 | yes |
| b_z0550_t00_a000_x2y2 | predicted_region | 35048 | 0.0167 | 0.0224 | 0.0627 | yes |
| b_z0550_t15_a000_x2y2 | predicted_region | 33406 | 0.0163 | 0.0247 | -0.0663 | yes |
| b_z0550_t15_a090_x2y2 | predicted_region | 30294 | 0.0174 | 0.0156 | -0.0903 | yes |
| b_z0550_t15_a180_x2y2 | predicted_region | 32846 | 0.0167 | 0.0184 | 0.0249 | yes |
| b_z0550_t15_a270_x2y2 | predicted_region | 33073 | 0.0167 | 0.0254 | 0.0424 | yes |
| b_z0550_t30_a000_x2y2 | predicted_region | 12714 | 0.0170 | 0.0274 | 0.1542 | yes |
| b_z0550_t30_a090_x2y2 | predicted_region | 13511 | 0.0165 | 0.0194 | 0.1653 | yes |
| b_z0750_t00_a000_x0y0 | predicted_region | 18207 | 0.0314 | 0.0237 | -0.0758 | yes |
| b_z0750_t15_a000_x0y0 | predicted_region | 14733 | 0.0307 | 0.0283 | -0.1056 | yes |
| b_z0750_t15_a090_x0y0 | predicted_region | 15171 | 0.0301 | 0.0250 | -0.1854 | yes |
| b_z0750_t15_a180_x0y0 | predicted_region | 14460 | 0.0312 | 0.0188 | -0.0353 | yes |
| b_z0750_t15_a270_x0y0 | predicted_region | 16037 | 0.0314 | 0.0254 | 0.0097 | yes |
| b_z0750_t30_a000_x0y0 | predicted_region | 4655 | 0.0323 | 0.0254 | 0.1866 | yes |
| b_z0750_t30_a090_x0y0 | predicted_region | 6350 | 0.0283 | 0.0273 | -0.2858 | yes |
| b_z0750_t30_a180_x0y0 | predicted_region | 7393 | 0.0335 | 0.0111 | -0.0043 | yes |
| b_z0750_t30_a270_x0y0 | predicted_region | 8175 | 0.0319 | 0.0246 | -0.0760 | yes |
| b_z0750_t00_a000_x1y0 | predicted_region | 17737 | 0.0300 | 0.0225 | 0.0875 | yes |
| b_z0750_t15_a000_x1y0 | predicted_region | 12602 | 0.0297 | 0.0270 | -0.0780 | yes |
| b_z0750_t15_a090_x1y0 | predicted_region | 15409 | 0.0294 | 0.0229 | -0.1567 | yes |
| b_z0750_t15_a180_x1y0 | predicted_region | 16057 | 0.0307 | 0.0248 | 0.0413 | yes |
| b_z0750_t15_a270_x1y0 | predicted_region | 15781 | 0.0295 | 0.0245 | -0.0055 | yes |
| b_z0750_t30_a000_x1y0 | predicted_region | 3851 | 0.0287 | 0.0287 | -0.0957 | yes |
| b_z0750_t30_a090_x1y0 | predicted_region | 7024 | 0.0278 | 0.0225 | -0.2421 | yes |
| b_z0750_t30_a180_x1y0 | predicted_region | 4287 | 0.0314 | 0.0098 | 0.0324 | yes |
| b_z0750_t30_a270_x1y0 | predicted_region | 7072 | 0.0290 | 0.0316 | -0.1281 | yes |
| b_z0750_t00_a000_x2y0 | predicted_region | 18211 | 0.0300 | 0.0232 | -0.0335 | yes |
| b_z0750_t15_a000_x2y0 | predicted_region | 16115 | 0.0310 | 0.0286 | 0.1211 | yes |
| b_z0750_t15_a090_x2y0 | predicted_region | 15116 | 0.0310 | 0.0256 | 0.1700 | yes |
| b_z0750_t15_a180_x2y0 | predicted_region | 15534 | 0.0302 | 0.0249 | 0.0104 | yes |
| b_z0750_t15_a270_x2y0 | predicted_region | 15195 | 0.0313 | 0.0289 | -0.0189 | yes |
| b_z0750_t30_a000_x2y0 | predicted_region | 5563 | 0.0336 | 0.0296 | 0.1561 | yes |
| b_z0750_t30_a090_x2y0 | predicted_region | 7208 | 0.0298 | 0.0256 | -0.2485 | yes |
| b_z0750_t30_a180_x2y0 | predicted_region | 4302 | 0.0327 | 0.0226 | 0.1090 | yes |
| b_z0750_t30_a270_x2y0 | predicted_region | 7451 | 0.0304 | 0.0329 | -0.1287 | yes |
| b_z0750_t00_a000_x2y1 | predicted_region | 18011 | 0.0298 | 0.0250 | -0.0297 | yes |
| b_z0750_t15_a000_x2y1 | predicted_region | 16217 | 0.0321 | 0.0254 | -0.0833 | yes |
| b_z0750_t15_a090_x2y1 | predicted_region | 15535 | 0.0304 | 0.0211 | -0.1260 | yes |
| b_z0750_t15_a180_x2y1 | predicted_region | 15101 | 0.0313 | 0.0205 | 0.0352 | yes |
| b_z0750_t15_a270_x2y1 | predicted_region | 16277 | 0.0305 | 0.0263 | 0.0722 | yes |
| b_z0750_t30_a000_x2y1 | predicted_region | 5782 | 0.0307 | 0.0266 | 0.1932 | yes |
| b_z0750_t30_a090_x2y1 | predicted_region | 4899 | 0.0314 | 0.0244 | 0.2421 | yes |
| b_z0750_t30_a180_x2y1 | predicted_region | 3832 | 0.0316 | 0.0106 | -0.0263 | yes |
| b_z0750_t30_a270_x2y1 | predicted_region | 4183 | 0.0301 | 0.0365 | 0.1826 | yes |
| b_z0750_t00_a000_x1y1 | predicted_region | 17427 | 0.0301 | 0.0207 | -0.0490 | yes |
| b_z0750_t15_a000_x1y1 | predicted_region | 14535 | 0.0304 | 0.0295 | 0.1318 | yes |
| b_z0750_t15_a090_x1y1 | predicted_region | 15487 | 0.0316 | 0.0181 | -0.1381 | yes |
| b_z0750_t15_a180_x1y1 | predicted_region | 15232 | 0.0296 | 0.0177 | 0.0024 | yes |
| b_z0750_t15_a270_x1y1 | predicted_region | 14742 | 0.0293 | 0.0244 | 0.0106 | yes |
| b_z0750_t30_a000_x1y1 | predicted_region | 7536 | 0.0328 | 0.0327 | -0.0996 | yes |
| b_z0750_t30_a090_x1y1 | predicted_region | 3737 | 0.0276 | 0.0168 | 0.1331 | yes |
| b_z0750_t30_a180_x1y1 | predicted_region | 1713 | 0.0295 | 0.0088 | -0.0228 | yes |
| b_z0750_t30_a270_x1y1 | predicted_region | 8489 | 0.0284 | 0.0276 | -0.0607 | yes |
| b_z0750_t00_a000_x0y1 | predicted_region | 18015 | 0.0309 | 0.0243 | 0.0881 | yes |
| b_z0750_t15_a000_x0y1 | predicted_region | 15889 | 0.0293 | 0.0241 | -0.0937 | yes |
| b_z0750_t15_a090_x0y1 | predicted_region | 16281 | 0.0300 | 0.0211 | 0.1587 | yes |
| b_z0750_t15_a180_x0y1 | predicted_region | 16445 | 0.0306 | 0.0134 | -0.0376 | yes |
| b_z0750_t15_a270_x0y1 | predicted_region | 15221 | 0.0310 | 0.0246 | 0.0143 | yes |
| b_z0750_t30_a000_x0y1 | predicted_region | 7031 | 0.0315 | 0.0269 | -0.0905 | yes |
| b_z0750_t30_a090_x0y1 | predicted_region | 2679 | 0.0293 | 0.0195 | -0.2338 | yes |
| b_z0750_t30_a180_x0y1 | predicted_region | 3183 | 0.0333 | 0.0013 | -0.0401 | yes |
| b_z0750_t30_a270_x0y1 | predicted_region | 5738 | 0.0296 | 0.0262 | -0.0855 | yes |
| b_z0750_t00_a000_x0y2 | predicted_region | 18207 | 0.0308 | 0.0161 | -0.0684 | yes |
| b_z0750_t15_a000_x0y2 | predicted_region | 17205 | 0.0301 | 0.0252 | 0.1010 | yes |
| b_z0750_t15_a090_x0y2 | predicted_region | 15825 | 0.0322 | 0.0159 | -0.1473 | yes |
| b_z0750_t15_a180_x0y2 | predicted_region | 15764 | 0.0306 | 0.0073 | -0.0378 | yes |
| b_z0750_t15_a270_x0y2 | predicted_region | 15792 | 0.0308 | 0.0188 | 0.0207 | yes |
| b_z0750_t30_a000_x0y2 | predicted_region | 4737 | 0.0317 | 0.0267 | 0.1269 | yes |
| b_z0750_t30_a090_x0y2 | predicted_region | 7430 | 0.0321 | 0.0229 | -0.2520 | yes |
| b_z0750_t30_a180_x0y2 | predicted_region | 4707 | 0.0299 | 0.0135 | 0.0491 | yes |
| b_z0750_t30_a270_x0y2 | predicted_region | 6139 | 0.0318 | 0.0227 | 0.0984 | yes |
| b_z0750_t00_a000_x1y2 | predicted_region | 17734 | 0.0312 | 0.0150 | -0.0474 | yes |
| b_z0750_t15_a000_x1y2 | predicted_region | 15418 | 0.0300 | 0.0242 | -0.0970 | yes |
| b_z0750_t15_a090_x1y2 | predicted_region | 14926 | 0.0300 | 0.0120 | -0.1170 | yes |
| b_z0750_t15_a180_x1y2 | predicted_region | 15600 | 0.0309 | 0.0133 | 0.0031 | yes |
| b_z0750_t15_a270_x1y2 | predicted_region | 15275 | 0.0295 | 0.0228 | 0.0306 | yes |
| b_z0750_t30_a000_x1y2 | predicted_region | 5180 | 0.0311 | 0.0263 | 0.1642 | yes |
| b_z0750_t30_a090_x1y2 | predicted_region | 6769 | 0.0296 | 0.0205 | 0.2031 | yes |
| b_z0750_t30_a180_x1y2 | predicted_region | 7127 | 0.0311 | 0.0094 | 0.0324 | yes |
| b_z0750_t30_a270_x1y2 | predicted_region | 4257 | 0.0289 | 0.0299 | -0.0679 | yes |
| b_z0750_t00_a000_x2y2 | predicted_region | 18208 | 0.0314 | 0.0184 | -0.0228 | yes |
| b_z0750_t15_a000_x2y2 | predicted_region | 16075 | 0.0315 | 0.0289 | -0.0869 | yes |
| b_z0750_t15_a090_x2y2 | predicted_region | 16872 | 0.0306 | 0.0172 | -0.1064 | yes |
| b_z0750_t15_a180_x2y2 | predicted_region | 15467 | 0.0308 | 0.0207 | -0.0176 | yes |
| b_z0750_t15_a270_x2y2 | predicted_region | 15791 | 0.0308 | 0.0249 | -0.0110 | yes |
| b_z0750_t30_a000_x2y2 | predicted_region | 766 | 0.0319 | 0.0327 | 0.1528 | yes |
| b_z0750_t30_a090_x2y2 | predicted_region | 6983 | 0.0314 | 0.0136 | -0.1715 | yes |
| b_z0750_t30_a180_x2y2 | predicted_region | 8157 | 0.0307 | 0.0161 | -0.0874 | yes |
| b_z0750_t30_a270_x2y2 | predicted_region | 5591 | 0.0311 | 0.0299 | -0.0955 | yes |
| b_z0950_t00_a000_x0y0 | predicted_region | 10862 | 0.0499 | 0.0279 | 0.0945 | yes |
| b_z0950_t15_a000_x0y0 | predicted_region | 5753 | 0.0486 | 0.0272 | -0.0639 | yes |
| b_z0950_t15_a090_x0y0 | predicted_region | 6850 | 0.0475 | 0.0356 | 0.2297 | yes |
| b_z0950_t15_a180_x0y0 | predicted_region | 8360 | 0.0473 | 0.0295 | 0.0095 | yes |
| b_z0950_t15_a270_x0y0 | predicted_region | 8884 | 0.0476 | 0.0300 | -0.0095 | yes |
| b_z0950_t30_a000_x0y0 | predicted_region | 4366 | 0.0524 | 0.0375 | 0.2488 | yes |
| b_z0950_t30_a090_x0y0 | predicted_region | 5186 | 0.0513 | 0.0249 | 0.2958 | yes |
| b_z0950_t30_a180_x0y0 | predicted_region | 517 | 0.0454 | 0.0702 | -0.4805 | yes |
| b_z0950_t30_a270_x0y0 | predicted_region | 1110 | 0.0471 | 0.0654 | -0.2748 | yes |
| b_z0950_t00_a000_x1y0 | predicted_region | 10668 | 0.0462 | 0.0279 | 0.0903 | yes |
| b_z0950_t15_a000_x1y0 | predicted_region | 6716 | 0.0482 | 0.0295 | 0.1266 | yes |
| b_z0950_t15_a090_x1y0 | predicted_region | 8679 | 0.0486 | 0.0297 | 0.2228 | yes |
| b_z0950_t15_a180_x1y0 | predicted_region | 8917 | 0.0470 | 0.0147 | -0.0491 | yes |
| b_z0950_t15_a270_x1y0 | predicted_region | 9401 | 0.0459 | 0.0263 | 0.0686 | yes |
| b_z0950_t30_a000_x1y0 | predicted_region | 1944 | 0.0513 | 0.0162 | 0.0155 | yes |
| b_z0950_t30_a090_x1y0 | predicted_region | 4623 | 0.0468 | 0.0270 | 0.3054 | yes |
| b_z0950_t30_a180_x1y0 | predicted_region | 1984 | 0.0517 | 0.0130 | 0.0146 | yes |
| b_z0950_t30_a270_x1y0 | predicted_region | 918 | 0.0516 | 0.0455 | 0.3190 | yes |
| b_z0950_t00_a000_x2y0 | predicted_region | 10830 | 0.0505 | 0.0254 | -0.0330 | yes |
| b_z0950_t15_a000_x2y0 | predicted_region | 10493 | 0.0506 | 0.0290 | 0.1513 | yes |
| b_z0950_t15_a090_x2y0 | predicted_region | 9782 | 0.0482 | 0.0341 | 0.2107 | yes |
| b_z0950_t15_a180_x2y0 | predicted_region | 9100 | 0.0495 | 0.0223 | 0.0017 | yes |
| b_z0950_t15_a270_x2y0 | predicted_region | 8446 | 0.0477 | 0.0306 | -0.0535 | yes |
| b_z0950_t30_a000_x2y0 | predicted_region | 725 | 0.0408 | 0.1659 | -1.2699 | yes |
| b_z0950_t30_a090_x2y0 | predicted_region | 1132 | 0.0509 | 0.0320 | -0.2451 | yes |
| b_z0950_t30_a180_x2y0 | predicted_region | 1348 | 0.0498 | 0.0085 | 0.0072 | yes |
| b_z0950_t30_a270_x2y0 | predicted_region | 1604 | 0.0469 | 0.0369 | -0.2171 | yes |
| b_z0950_t00_a000_x2y1 | predicted_region | 10839 | 0.0478 | 0.0208 | -0.0257 | yes |
| b_z0950_t15_a000_x2y1 | predicted_region | 8904 | 0.0478 | 0.0278 | -0.1017 | yes |
| b_z0950_t15_a090_x2y1 | predicted_region | 9111 | 0.0479 | 0.0191 | -0.1371 | yes |
| b_z0950_t15_a180_x2y1 | predicted_region | 8592 | 0.0467 | 0.0189 | 0.0379 | yes |
| b_z0950_t15_a270_x2y1 | predicted_region | 8594 | 0.0480 | 0.0217 | 0.0844 | yes |
| b_z0950_t30_a000_x2y1 | predicted_region | 1747 | 0.0451 | 0.0102 | 0.1005 | yes |
| b_z0950_t30_a090_x2y1 | predicted_region | 4612 | 0.0515 | 0.0208 | 0.2552 | yes |
| b_z0950_t30_a180_x2y1 | predicted_region | 490 | 0.0433 | 0.1010 | -0.2938 | yes |
| b_z0950_t30_a270_x2y1 | predicted_region | 2967 | 0.0498 | 0.0298 | 0.1988 | yes |
| b_z0950_t00_a000_x1y1 | predicted_region | 10701 | 0.0484 | 0.0264 | 0.0829 | yes |
| b_z0950_t15_a000_x1y1 | predicted_region | 8610 | 0.0488 | 0.0270 | 0.1424 | yes |
| b_z0950_t15_a090_x1y1 | predicted_region | 8735 | 0.0463 | 0.0207 | -0.1629 | yes |
| b_z0950_t15_a180_x1y1 | predicted_region | 8656 | 0.0476 | 0.0179 | 0.0255 | yes |
| b_z0950_t15_a270_x1y1 | predicted_region | 7096 | 0.0462 | 0.0232 | 0.0395 | yes |
| b_z0950_t30_a000_x1y1 | predicted_region | 2998 | 0.0502 | 0.0447 | 0.2924 | yes |
| b_z0950_t30_a090_x1y1 | predicted_region | 2548 | 0.0456 | 0.0229 | -0.2833 | yes |
| b_z0950_t30_a180_x1y1 | predicted_region | 1753 | 0.0427 | 0.0067 | -0.0146 | yes |
| b_z0950_t30_a270_x1y1 | predicted_region | 3270 | 0.0453 | 0.0270 | -0.1393 | yes |
| b_z0950_t00_a000_x0y1 | predicted_region | 10840 | 0.0469 | 0.0236 | 0.0913 | yes |
| b_z0950_t15_a000_x0y1 | predicted_region | 7944 | 0.0485 | 0.0285 | 0.1612 | yes |
| b_z0950_t15_a090_x0y1 | predicted_region | 9376 | 0.0482 | 0.0217 | 0.1691 | yes |
| b_z0950_t15_a180_x0y1 | predicted_region | 9458 | 0.0504 | 0.0147 | -0.0830 | yes |
| b_z0950_t15_a270_x0y1 | predicted_region | 9852 | 0.0470 | 0.0263 | 0.0474 | yes |
| b_z0950_t30_a000_x0y1 | predicted_region | 1484 | 0.0474 | 0.0229 | -0.1284 | yes |
| b_z0950_t30_a090_x0y1 | predicted_region | 2902 | 0.0486 | 0.0408 | 0.4152 | yes |
| b_z0950_t30_a180_x0y1 | predicted_region | 553 | 0.0444 | 0.0144 | -0.0500 | yes |
| b_z0950_t30_a270_x0y1 | predicted_region | 935 | 0.0501 | 0.0537 | -0.3886 | yes |
| b_z0950_t00_a000_x0y2 | predicted_region | 10971 | 0.0483 | 0.0187 | 0.0773 | yes |
| b_z0950_t15_a000_x0y2 | predicted_region | 8172 | 0.0481 | 0.0249 | 0.1411 | yes |
| b_z0950_t15_a090_x0y2 | predicted_region | 9079 | 0.0515 | 0.0206 | -0.1758 | yes |
| b_z0950_t15_a180_x0y2 | predicted_region | 6569 | 0.0498 | 0.0151 | -0.1128 | yes |
| b_z0950_t15_a270_x0y2 | predicted_region | 8939 | 0.0507 | 0.0149 | 0.0258 | yes |
| b_z0950_t30_a000_x0y2 | predicted_region | 660 | 0.0505 | 0.0506 | -0.1033 | yes |
| b_z0950_t30_a090_x0y2 | predicted_region | 5087 | 0.0534 | 0.0108 | -0.1782 | yes |
| b_z0950_t30_a180_x0y2 | predicted_region | 4059 | 0.0510 | 0.0059 | -0.0202 | yes |
| b_z0950_t30_a270_x0y2 | predicted_region | 3837 | 0.0504 | 0.0229 | 0.1422 | yes |
| b_z0950_t00_a000_x1y2 | predicted_region | 10586 | 0.0465 | 0.0260 | -0.0498 | yes |
| b_z0950_t15_a000_x1y2 | predicted_region | 10128 | 0.0492 | 0.0244 | 0.1355 | yes |
| b_z0950_t15_a090_x1y2 | predicted_region | 8897 | 0.0486 | 0.0225 | 0.1556 | yes |
| b_z0950_t15_a180_x1y2 | predicted_region | 9317 | 0.0488 | 0.0092 | -0.0168 | yes |
| b_z0950_t15_a270_x1y2 | predicted_region | 8860 | 0.0497 | 0.0236 | 0.0563 | yes |
| b_z0950_t30_a000_x1y2 | predicted_region | 2114 | 0.0490 | 0.0486 | -0.3266 | yes |
| b_z0950_t30_a090_x1y2 | predicted_region | 3212 | 0.0435 | 0.0204 | 0.2411 | yes |
| b_z0950_t30_a180_x1y2 | predicted_region | 1622 | 0.0456 | 0.0378 | 0.3103 | yes |
| b_z0950_t30_a270_x1y2 | predicted_region | 3410 | 0.0467 | 0.0224 | -0.0424 | yes |
| b_z0950_t00_a000_x2y2 | predicted_region | 10968 | 0.0500 | 0.0203 | -0.0220 | yes |
| b_z0950_t15_a000_x2y2 | predicted_region | 9262 | 0.0510 | 0.0233 | 0.1311 | yes |
| b_z0950_t15_a090_x2y2 | predicted_region | 8826 | 0.0510 | 0.0199 | 0.1384 | yes |
| b_z0950_t15_a180_x2y2 | predicted_region | 9041 | 0.0476 | 0.0235 | -0.0475 | yes |
| b_z0950_t15_a270_x2y2 | predicted_region | 8853 | 0.0493 | 0.0246 | 0.1037 | yes |
| b_z0950_t30_a000_x2y2 | predicted_region | 706 | 0.0480 | 0.0444 | -0.2668 | yes |
| b_z0950_t30_a090_x2y2 | predicted_region | 5003 | 0.0486 | 0.0133 | -0.1998 | yes |
| b_z0950_t30_a180_x2y2 | predicted_region | 886 | 0.0450 | 0.0650 | -0.0878 | yes |
| b_z0950_t30_a270_x2y2 | predicted_region | 511 | 0.0452 | 0.0658 | 0.0874 | yes |

### Errors against the known truth

| pass | rotation error (deg) | translation error (mm) | scale | scale error |
|---|---|---|---|---|
| pass1 | 0.0037 | 0.0335 | 1.000000 | 0.000000 |
| pass2 | 0.0038 | 0.0339 | 1.000000 | 0.000000 |

## Model: similarity

Verdict: NOT ACCEPTED. Status ResidualsExceedThreshold: registration solved, but a residual exceeds its acceptance threshold. Final pass: pass2; 235 of 235 poses used.

### Transform (sensor to base)

```
  0.999966  -0.000064  -0.000017 799.971975
 -0.000064  -0.999966  -0.000002  -0.010708
 -0.000017   0.000002  -0.999966 1199.988851
  0.000000   0.000000   0.000000   1.000000
```

Rotation vector 179.9999, -0.0057, -0.0015 deg; translation 799.9720, -0.0107, 1199.9889 mm; scale 0.999966.

### Residuals against the acceptance thresholds

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0286 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.1659 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1586 | 0.5000 | within the limit |
| maximum offset residual (mm) | 1.2607 | 1.0000 | EXCEEDS the limit |

### Spreads of the pose set

| quantity | value | minimum | result |
|---|---|---|---|
| normal spread | 0.2850 | 0.0500 | at or above the minimum |
| similarity spread | 0.1306 | 0.0100 | at or above the minimum |

### Passes

| pass | status (candidate regions from) | poses used | RMS normal (deg) | RMS offset (mm) |
|---|---|---|---|---|
| pass1 | ResidualsExceedThreshold (sensor_in_base) | 235/235 | 0.0286 | 0.1590 |
| pass2 | ResidualsExceedThreshold (pass1_transform) | 235/235 | 0.0286 | 0.1586 |

### Segmentation

Segmentation methods: predicted_region 235.
Mask pixels per pose: minimum 490, median 10972, maximum 35055.
Poses whose segmentation failed: none.
Poses rejected as outliers: none.

### Residual of every pose

| pose | method | pixels | plane RMS (mm) | normal (deg) | offset (mm) | used |
|---|---|---|---|---|---|---|
| b_z0550_t00_a000_x0y0 | predicted_region | 35055 | 0.0170 | 0.0238 | 0.0881 | yes |
| b_z0550_t15_a000_x0y0 | predicted_region | 31635 | 0.0169 | 0.0286 | -0.0985 | yes |
| b_z0550_t15_a090_x0y0 | predicted_region | 31358 | 0.0171 | 0.0225 | -0.1567 | yes |
| b_z0550_t15_a180_x0y0 | predicted_region | 31371 | 0.0166 | 0.0205 | 0.0584 | yes |
| b_z0550_t15_a270_x0y0 | predicted_region | 30641 | 0.0167 | 0.0252 | 0.0223 | yes |
| b_z0550_t30_a180_x0y0 | predicted_region | 12492 | 0.0164 | 0.0117 | -0.0281 | yes |
| b_z0550_t30_a270_x0y0 | predicted_region | 13141 | 0.0163 | 0.0294 | 0.0890 | yes |
| b_z0550_t00_a000_x1y0 | predicted_region | 33850 | 0.0160 | 0.0240 | -0.0620 | yes |
| b_z0550_t15_a000_x1y0 | predicted_region | 29216 | 0.0168 | 0.0300 | -0.0890 | yes |
| b_z0550_t15_a090_x1y0 | predicted_region | 29916 | 0.0161 | 0.0237 | 0.1445 | yes |
| b_z0550_t15_a180_x1y0 | predicted_region | 28977 | 0.0165 | 0.0214 | -0.0278 | yes |
| b_z0550_t15_a270_x1y0 | predicted_region | 31111 | 0.0165 | 0.0278 | 0.0234 | yes |
| b_z0550_t30_a000_x1y0 | predicted_region | 14285 | 0.0160 | 0.0293 | 0.1354 | yes |
| b_z0550_t30_a090_x1y0 | predicted_region | 12906 | 0.0154 | 0.0250 | 0.2067 | yes |
| b_z0550_t30_a180_x1y0 | predicted_region | 12591 | 0.0166 | 0.0145 | 0.0039 | yes |
| b_z0550_t30_a270_x1y0 | predicted_region | 13911 | 0.0165 | 0.0282 | 0.1014 | yes |
| b_z0550_t00_a000_x2y0 | predicted_region | 34823 | 0.0162 | 0.0272 | -0.0376 | yes |
| b_z0550_t15_a000_x2y0 | predicted_region | 30460 | 0.0168 | 0.0299 | -0.0731 | yes |
| b_z0550_t15_a090_x2y0 | predicted_region | 32656 | 0.0168 | 0.0247 | -0.1231 | yes |
| b_z0550_t15_a180_x2y0 | predicted_region | 31424 | 0.0165 | 0.0223 | 0.0281 | yes |
| b_z0550_t15_a270_x2y0 | predicted_region | 31090 | 0.0166 | 0.0279 | -0.0055 | yes |
| b_z0550_t30_a090_x2y0 | predicted_region | 15089 | 0.0166 | 0.0246 | 0.2041 | yes |
| b_z0550_t30_a180_x2y0 | predicted_region | 11959 | 0.0167 | 0.0192 | 0.0403 | yes |
| b_z0550_t00_a000_x2y1 | predicted_region | 34274 | 0.0164 | 0.0249 | -0.0355 | yes |
| b_z0550_t15_a000_x2y1 | predicted_region | 29607 | 0.0167 | 0.0275 | -0.0751 | yes |
| b_z0550_t15_a090_x2y1 | predicted_region | 29449 | 0.0164 | 0.0223 | 0.1243 | yes |
| b_z0550_t15_a180_x2y1 | predicted_region | 31153 | 0.0172 | 0.0213 | 0.0183 | yes |
| b_z0550_t15_a270_x2y1 | predicted_region | 32250 | 0.0162 | 0.0272 | 0.0442 | yes |
| b_z0550_t30_a000_x2y1 | predicted_region | 12586 | 0.0165 | 0.0305 | -0.1023 | yes |
| b_z0550_t30_a090_x2y1 | predicted_region | 13306 | 0.0168 | 0.0179 | -0.1772 | yes |
| b_z0550_t30_a180_x2y1 | predicted_region | 12351 | 0.0162 | 0.0180 | -0.0399 | yes |
| b_z0550_t30_a270_x2y1 | predicted_region | 14797 | 0.0156 | 0.0288 | -0.0750 | yes |
| b_z0550_t00_a000_x1y1 | predicted_region | 33227 | 0.0157 | 0.0219 | -0.0589 | yes |
| b_z0550_t15_a000_x1y1 | predicted_region | 31675 | 0.0163 | 0.0268 | -0.0906 | yes |
| b_z0550_t15_a090_x1y1 | predicted_region | 27608 | 0.0163 | 0.0223 | 0.1318 | yes |
| b_z0550_t15_a180_x1y1 | predicted_region | 31657 | 0.0162 | 0.0183 | 0.0366 | yes |
| b_z0550_t15_a270_x1y1 | predicted_region | 29994 | 0.0166 | 0.0260 | 0.0081 | yes |
| b_z0550_t30_a000_x1y1 | predicted_region | 5019 | 0.0154 | 0.0315 | -0.1254 | yes |
| b_z0550_t30_a090_x1y1 | predicted_region | 14964 | 0.0157 | 0.0163 | -0.1858 | yes |
| b_z0550_t30_a180_x1y1 | predicted_region | 11547 | 0.0160 | 0.0121 | 0.0090 | yes |
| b_z0550_t30_a270_x1y1 | predicted_region | 13346 | 0.0149 | 0.0255 | -0.0452 | yes |
| b_z0550_t00_a000_x0y1 | predicted_region | 34474 | 0.0165 | 0.0217 | 0.0817 | yes |
| b_z0550_t15_a000_x0y1 | predicted_region | 29709 | 0.0163 | 0.0252 | 0.1027 | yes |
| b_z0550_t15_a090_x0y1 | predicted_region | 32425 | 0.0161 | 0.0200 | -0.1486 | yes |
| b_z0550_t15_a180_x0y1 | predicted_region | 31534 | 0.0168 | 0.0161 | 0.0571 | yes |
| b_z0550_t15_a270_x0y1 | predicted_region | 30267 | 0.0165 | 0.0217 | -0.0074 | yes |
| b_z0550_t30_a000_x0y1 | predicted_region | 13522 | 0.0163 | 0.0277 | 0.1200 | yes |
| b_z0550_t30_a090_x0y1 | predicted_region | 15911 | 0.0156 | 0.0213 | -0.2067 | yes |
| b_z0550_t30_a180_x0y1 | predicted_region | 14952 | 0.0170 | 0.0078 | -0.0218 | yes |
| b_z0550_t30_a270_x0y1 | predicted_region | 13101 | 0.0165 | 0.0242 | 0.0667 | yes |
| b_z0550_t00_a000_x0y2 | predicted_region | 35032 | 0.0175 | 0.0148 | -0.0733 | yes |
| b_z0550_t15_a000_x0y2 | predicted_region | 33022 | 0.0168 | 0.0235 | 0.0990 | yes |
| b_z0550_t15_a090_x0y2 | predicted_region | 33474 | 0.0166 | 0.0181 | 0.1201 | yes |
| b_z0550_t15_a180_x0y2 | predicted_region | 30382 | 0.0172 | 0.0127 | 0.0458 | yes |
| b_z0550_t15_a270_x0y2 | predicted_region | 30477 | 0.0163 | 0.0175 | -0.0125 | yes |
| b_z0550_t30_a000_x0y2 | predicted_region | 13484 | 0.0170 | 0.0270 | -0.0835 | yes |
| b_z0550_t30_a270_x0y2 | predicted_region | 13364 | 0.0162 | 0.0211 | 0.0515 | yes |
| b_z0550_t00_a000_x1y2 | predicted_region | 34065 | 0.0160 | 0.0175 | -0.0545 | yes |
| b_z0550_t15_a000_x1y2 | predicted_region | 31010 | 0.0166 | 0.0250 | 0.1023 | yes |
| b_z0550_t15_a090_x1y2 | predicted_region | 28353 | 0.0164 | 0.0178 | 0.1144 | yes |
| b_z0550_t15_a180_x1y2 | predicted_region | 31187 | 0.0166 | 0.0128 | -0.0176 | yes |
| b_z0550_t15_a270_x1y2 | predicted_region | 31409 | 0.0159 | 0.0232 | 0.0109 | yes |
| b_z0550_t30_a000_x1y2 | predicted_region | 12556 | 0.0165 | 0.0271 | 0.1322 | yes |
| b_z0550_t30_a090_x1y2 | predicted_region | 14149 | 0.0163 | 0.0138 | -0.1654 | yes |
| b_z0550_t30_a180_x1y2 | predicted_region | 13560 | 0.0164 | 0.0114 | 0.0232 | yes |
| b_z0550_t30_a270_x1y2 | predicted_region | 13289 | 0.0154 | 0.0273 | 0.0854 | yes |
| b_z0550_t00_a000_x2y2 | predicted_region | 35048 | 0.0167 | 0.0224 | 0.0564 | yes |
| b_z0550_t15_a000_x2y2 | predicted_region | 33410 | 0.0163 | 0.0247 | -0.0736 | yes |
| b_z0550_t15_a090_x2y2 | predicted_region | 30295 | 0.0174 | 0.0156 | -0.0979 | yes |
| b_z0550_t15_a180_x2y2 | predicted_region | 32848 | 0.0167 | 0.0184 | 0.0200 | yes |
| b_z0550_t15_a270_x2y2 | predicted_region | 33077 | 0.0167 | 0.0254 | 0.0378 | yes |
| b_z0550_t30_a000_x2y2 | predicted_region | 12715 | 0.0170 | 0.0274 | 0.1464 | yes |
| b_z0550_t30_a090_x2y2 | predicted_region | 13512 | 0.0165 | 0.0194 | 0.1568 | yes |
| b_z0750_t00_a000_x0y0 | predicted_region | 18208 | 0.0314 | 0.0237 | -0.0751 | yes |
| b_z0750_t15_a000_x0y0 | predicted_region | 14734 | 0.0307 | 0.0283 | -0.1037 | yes |
| b_z0750_t15_a090_x0y0 | predicted_region | 15172 | 0.0301 | 0.0250 | -0.1831 | yes |
| b_z0750_t15_a180_x0y0 | predicted_region | 14462 | 0.0312 | 0.0188 | -0.0358 | yes |
| b_z0750_t15_a270_x0y0 | predicted_region | 16038 | 0.0314 | 0.0254 | 0.0089 | yes |
| b_z0750_t30_a000_x0y0 | predicted_region | 4655 | 0.0323 | 0.0254 | 0.1896 | yes |
| b_z0750_t30_a090_x0y0 | predicted_region | 6350 | 0.0283 | 0.0273 | -0.2822 | yes |
| b_z0750_t30_a180_x0y0 | predicted_region | 7395 | 0.0335 | 0.0111 | -0.0061 | yes |
| b_z0750_t30_a270_x0y0 | predicted_region | 8173 | 0.0319 | 0.0246 | -0.0782 | yes |
| b_z0750_t00_a000_x1y0 | predicted_region | 17738 | 0.0300 | 0.0225 | 0.0869 | yes |
| b_z0750_t15_a000_x1y0 | predicted_region | 12602 | 0.0297 | 0.0270 | -0.0774 | yes |
| b_z0750_t15_a090_x1y0 | predicted_region | 15410 | 0.0294 | 0.0229 | -0.1573 | yes |
| b_z0750_t15_a180_x1y0 | predicted_region | 16058 | 0.0307 | 0.0248 | 0.0396 | yes |
| b_z0750_t15_a270_x1y0 | predicted_region | 15782 | 0.0295 | 0.0245 | -0.0060 | yes |
| b_z0750_t30_a000_x1y0 | predicted_region | 3851 | 0.0287 | 0.0287 | -0.0939 | yes |
| b_z0750_t30_a090_x1y0 | predicted_region | 7023 | 0.0278 | 0.0225 | -0.2426 | yes |
| b_z0750_t30_a180_x1y0 | predicted_region | 4287 | 0.0314 | 0.0098 | 0.0296 | yes |
| b_z0750_t30_a270_x1y0 | predicted_region | 4870 | 0.0288 | 0.0317 | -0.1300 | yes |
| b_z0750_t00_a000_x2y0 | predicted_region | 18212 | 0.0300 | 0.0232 | -0.0327 | yes |
| b_z0750_t15_a000_x2y0 | predicted_region | 16117 | 0.0310 | 0.0286 | 0.1230 | yes |
| b_z0750_t15_a090_x2y0 | predicted_region | 15117 | 0.0310 | 0.0256 | 0.1691 | yes |
| b_z0750_t15_a180_x2y0 | predicted_region | 15536 | 0.0302 | 0.0249 | 0.0100 | yes |
| b_z0750_t15_a270_x2y0 | predicted_region | 15199 | 0.0313 | 0.0289 | -0.0166 | yes |
| b_z0750_t30_a000_x2y0 | predicted_region | 5565 | 0.0336 | 0.0295 | 0.1589 | yes |
| b_z0750_t30_a090_x2y0 | predicted_region | 7208 | 0.0298 | 0.0256 | -0.2508 | yes |
| b_z0750_t30_a180_x2y0 | predicted_region | 4302 | 0.0327 | 0.0227 | 0.1076 | yes |
| b_z0750_t30_a270_x2y0 | predicted_region | 7453 | 0.0304 | 0.0328 | -0.1250 | yes |
| b_z0750_t00_a000_x2y1 | predicted_region | 18012 | 0.0298 | 0.0250 | -0.0297 | yes |
| b_z0750_t15_a000_x2y1 | predicted_region | 16217 | 0.0321 | 0.0254 | -0.0833 | yes |
| b_z0750_t15_a090_x2y1 | predicted_region | 15535 | 0.0304 | 0.0211 | -0.1275 | yes |
| b_z0750_t15_a180_x2y1 | predicted_region | 15101 | 0.0313 | 0.0205 | 0.0352 | yes |
| b_z0750_t15_a270_x2y1 | predicted_region | 16279 | 0.0305 | 0.0263 | 0.0738 | yes |
| b_z0750_t30_a000_x2y1 | predicted_region | 5782 | 0.0307 | 0.0266 | 0.1931 | yes |
| b_z0750_t30_a090_x2y1 | predicted_region | 4900 | 0.0314 | 0.0244 | 0.2389 | yes |
| b_z0750_t30_a180_x2y1 | predicted_region | 3832 | 0.0316 | 0.0106 | -0.0263 | yes |
| b_z0750_t30_a270_x2y1 | predicted_region | 4183 | 0.0301 | 0.0365 | 0.1857 | yes |
| b_z0750_t00_a000_x1y1 | predicted_region | 17427 | 0.0301 | 0.0207 | -0.0504 | yes |
| b_z0750_t15_a000_x1y1 | predicted_region | 14538 | 0.0304 | 0.0295 | 0.1304 | yes |
| b_z0750_t15_a090_x1y1 | predicted_region | 15488 | 0.0316 | 0.0181 | -0.1394 | yes |
| b_z0750_t15_a180_x1y1 | predicted_region | 15232 | 0.0296 | 0.0177 | 0.0010 | yes |
| b_z0750_t15_a270_x1y1 | predicted_region | 14744 | 0.0293 | 0.0244 | 0.0093 | yes |
| b_z0750_t30_a000_x1y1 | predicted_region | 7548 | 0.0328 | 0.0327 | -0.1008 | yes |
| b_z0750_t30_a090_x1y1 | predicted_region | 3737 | 0.0276 | 0.0168 | 0.1319 | yes |
| b_z0750_t30_a180_x1y1 | predicted_region | 1713 | 0.0295 | 0.0088 | -0.0240 | yes |
| b_z0750_t30_a270_x1y1 | predicted_region | 8490 | 0.0284 | 0.0277 | -0.0619 | yes |
| b_z0750_t00_a000_x0y1 | predicted_region | 18016 | 0.0309 | 0.0243 | 0.0881 | yes |
| b_z0750_t15_a000_x0y1 | predicted_region | 15892 | 0.0293 | 0.0241 | -0.0937 | yes |
| b_z0750_t15_a090_x0y1 | predicted_region | 16281 | 0.0300 | 0.0211 | 0.1603 | yes |
| b_z0750_t15_a180_x0y1 | predicted_region | 16446 | 0.0306 | 0.0134 | -0.0376 | yes |
| b_z0750_t15_a270_x0y1 | predicted_region | 15221 | 0.0310 | 0.0246 | 0.0128 | yes |
| b_z0750_t30_a000_x0y1 | predicted_region | 7031 | 0.0315 | 0.0269 | -0.0905 | yes |
| b_z0750_t30_a090_x0y1 | predicted_region | 2679 | 0.0293 | 0.0195 | -0.2308 | yes |
| b_z0750_t30_a180_x0y1 | predicted_region | 3183 | 0.0333 | 0.0013 | -0.0401 | yes |
| b_z0750_t30_a270_x0y1 | predicted_region | 5752 | 0.0297 | 0.0263 | -0.0890 | yes |
| b_z0750_t00_a000_x0y2 | predicted_region | 18208 | 0.0308 | 0.0161 | -0.0676 | yes |
| b_z0750_t15_a000_x0y2 | predicted_region | 17206 | 0.0301 | 0.0252 | 0.1005 | yes |
| b_z0750_t15_a090_x0y2 | predicted_region | 15828 | 0.0322 | 0.0159 | -0.1451 | yes |
| b_z0750_t15_a180_x0y2 | predicted_region | 15765 | 0.0306 | 0.0073 | -0.0359 | yes |
| b_z0750_t15_a270_x0y2 | predicted_region | 15792 | 0.0308 | 0.0188 | 0.0199 | yes |
| b_z0750_t30_a000_x0y2 | predicted_region | 4737 | 0.0317 | 0.0267 | 0.1252 | yes |
| b_z0750_t30_a090_x0y2 | predicted_region | 7431 | 0.0321 | 0.0229 | -0.2483 | yes |
| b_z0750_t30_a180_x0y2 | predicted_region | 4709 | 0.0300 | 0.0136 | 0.0525 | yes |
| b_z0750_t30_a270_x0y2 | predicted_region | 6139 | 0.0318 | 0.0227 | 0.0961 | yes |
| b_z0750_t00_a000_x1y2 | predicted_region | 17735 | 0.0312 | 0.0150 | -0.0480 | yes |
| b_z0750_t15_a000_x1y2 | predicted_region | 15419 | 0.0300 | 0.0242 | -0.0987 | yes |
| b_z0750_t15_a090_x1y2 | predicted_region | 14929 | 0.0300 | 0.0120 | -0.1176 | yes |
| b_z0750_t15_a180_x1y2 | predicted_region | 15600 | 0.0309 | 0.0133 | 0.0037 | yes |
| b_z0750_t15_a270_x1y2 | predicted_region | 15277 | 0.0295 | 0.0228 | 0.0301 | yes |
| b_z0750_t30_a000_x1y2 | predicted_region | 5180 | 0.0311 | 0.0263 | 0.1613 | yes |
| b_z0750_t30_a090_x1y2 | predicted_region | 6770 | 0.0296 | 0.0205 | 0.2027 | yes |
| b_z0750_t30_a180_x1y2 | predicted_region | 7127 | 0.0311 | 0.0094 | 0.0342 | yes |
| b_z0750_t30_a270_x1y2 | predicted_region | 4258 | 0.0289 | 0.0299 | -0.0683 | yes |
| b_z0750_t00_a000_x2y2 | predicted_region | 18210 | 0.0314 | 0.0184 | -0.0221 | yes |
| b_z0750_t15_a000_x2y2 | predicted_region | 16076 | 0.0315 | 0.0289 | -0.0874 | yes |
| b_z0750_t15_a090_x2y2 | predicted_region | 16873 | 0.0306 | 0.0172 | -0.1072 | yes |
| b_z0750_t15_a180_x2y2 | predicted_region | 15467 | 0.0308 | 0.0207 | -0.0157 | yes |
| b_z0750_t15_a270_x2y2 | predicted_region | 15794 | 0.0308 | 0.0249 | -0.0087 | yes |
| b_z0750_t30_a000_x2y2 | predicted_region | 766 | 0.0319 | 0.0327 | 0.1511 | yes |
| b_z0750_t30_a090_x2y2 | predicted_region | 6983 | 0.0314 | 0.0136 | -0.1738 | yes |
| b_z0750_t30_a180_x2y2 | predicted_region | 8158 | 0.0307 | 0.0161 | -0.0845 | yes |
| b_z0750_t30_a270_x2y2 | predicted_region | 5591 | 0.0311 | 0.0299 | -0.0919 | yes |
| b_z0950_t00_a000_x0y0 | predicted_region | 10863 | 0.0499 | 0.0279 | 0.1024 | yes |
| b_z0950_t15_a000_x0y0 | predicted_region | 5754 | 0.0486 | 0.0272 | -0.0550 | yes |
| b_z0950_t15_a090_x0y0 | predicted_region | 6851 | 0.0475 | 0.0356 | 0.2388 | yes |
| b_z0950_t15_a180_x0y0 | predicted_region | 8363 | 0.0473 | 0.0295 | 0.0159 | yes |
| b_z0950_t15_a270_x0y0 | predicted_region | 8886 | 0.0476 | 0.0300 | -0.0034 | yes |
| b_z0950_t30_a000_x0y0 | predicted_region | 4366 | 0.0524 | 0.0375 | 0.2579 | yes |
| b_z0950_t30_a090_x0y0 | predicted_region | 5186 | 0.0513 | 0.0249 | 0.3055 | yes |
| b_z0950_t30_a180_x0y0 | predicted_region | 516 | 0.0455 | 0.0704 | -0.4778 | yes |
| b_z0950_t30_a270_x0y0 | predicted_region | 1109 | 0.0471 | 0.0653 | -0.2706 | yes |
| b_z0950_t00_a000_x1y0 | predicted_region | 10669 | 0.0462 | 0.0279 | 0.0967 | yes |
| b_z0950_t15_a000_x1y0 | predicted_region | 6717 | 0.0482 | 0.0295 | 0.1339 | yes |
| b_z0950_t15_a090_x1y0 | predicted_region | 8680 | 0.0486 | 0.0297 | 0.2290 | yes |
| b_z0950_t15_a180_x1y0 | predicted_region | 8918 | 0.0470 | 0.0147 | -0.0442 | yes |
| b_z0950_t15_a270_x1y0 | predicted_region | 9402 | 0.0459 | 0.0263 | 0.0747 | yes |
| b_z0950_t30_a000_x1y0 | predicted_region | 1944 | 0.0513 | 0.0162 | 0.0233 | yes |
| b_z0950_t30_a090_x1y0 | predicted_region | 4623 | 0.0468 | 0.0270 | 0.3108 | yes |
| b_z0950_t30_a180_x1y0 | predicted_region | 1984 | 0.0517 | 0.0130 | 0.0177 | yes |
| b_z0950_t30_a270_x1y0 | predicted_region | 918 | 0.0516 | 0.0455 | 0.3244 | yes |
| b_z0950_t00_a000_x2y0 | predicted_region | 10832 | 0.0505 | 0.0254 | -0.0251 | yes |
| b_z0950_t15_a000_x2y0 | predicted_region | 10493 | 0.0506 | 0.0290 | 0.1600 | yes |
| b_z0950_t15_a090_x2y0 | predicted_region | 9782 | 0.0482 | 0.0341 | 0.2167 | yes |
| b_z0950_t15_a180_x2y0 | predicted_region | 9100 | 0.0495 | 0.0223 | 0.0082 | yes |
| b_z0950_t15_a270_x2y0 | predicted_region | 8449 | 0.0477 | 0.0306 | -0.0444 | yes |
| b_z0950_t30_a000_x2y0 | predicted_region | 725 | 0.0408 | 0.1659 | -1.2607 | yes |
| b_z0950_t30_a090_x2y0 | predicted_region | 1132 | 0.0509 | 0.0320 | -0.2413 | yes |
| b_z0950_t30_a180_x2y0 | predicted_region | 1347 | 0.0498 | 0.0085 | 0.0116 | yes |
| b_z0950_t30_a270_x2y0 | predicted_region | 1604 | 0.0469 | 0.0369 | -0.2073 | yes |
| b_z0950_t00_a000_x2y1 | predicted_region | 10839 | 0.0478 | 0.0208 | -0.0187 | yes |
| b_z0950_t15_a000_x2y1 | predicted_region | 8906 | 0.0478 | 0.0278 | -0.0949 | yes |
| b_z0950_t15_a090_x2y1 | predicted_region | 9113 | 0.0479 | 0.0191 | -0.1319 | yes |
| b_z0950_t15_a180_x2y1 | predicted_region | 8593 | 0.0467 | 0.0189 | 0.0447 | yes |
| b_z0950_t15_a270_x2y1 | predicted_region | 8594 | 0.0480 | 0.0217 | 0.0927 | yes |
| b_z0950_t30_a000_x2y1 | predicted_region | 1747 | 0.0451 | 0.0102 | 0.1066 | yes |
| b_z0950_t30_a090_x2y1 | predicted_region | 4612 | 0.0515 | 0.0208 | 0.2582 | yes |
| b_z0950_t30_a180_x2y1 | predicted_region | 490 | 0.0433 | 0.1010 | -0.2877 | yes |
| b_z0950_t30_a270_x2y1 | predicted_region | 2967 | 0.0498 | 0.0298 | 0.2079 | yes |
| b_z0950_t00_a000_x1y1 | predicted_region | 10701 | 0.0484 | 0.0264 | 0.0883 | yes |
| b_z0950_t15_a000_x1y1 | predicted_region | 8610 | 0.0488 | 0.0270 | 0.1477 | yes |
| b_z0950_t15_a090_x1y1 | predicted_region | 8736 | 0.0463 | 0.0208 | -0.1578 | yes |
| b_z0950_t15_a180_x1y1 | predicted_region | 8656 | 0.0476 | 0.0179 | 0.0308 | yes |
| b_z0950_t15_a270_x1y1 | predicted_region | 7096 | 0.0462 | 0.0232 | 0.0447 | yes |
| b_z0950_t30_a000_x1y1 | predicted_region | 2998 | 0.0502 | 0.0447 | 0.2971 | yes |
| b_z0950_t30_a090_x1y1 | predicted_region | 2548 | 0.0456 | 0.0229 | -0.2785 | yes |
| b_z0950_t30_a180_x1y1 | predicted_region | 1753 | 0.0427 | 0.0067 | -0.0099 | yes |
| b_z0950_t30_a270_x1y1 | predicted_region | 3270 | 0.0453 | 0.0270 | -0.1345 | yes |
| b_z0950_t00_a000_x0y1 | predicted_region | 10840 | 0.0469 | 0.0236 | 0.0983 | yes |
| b_z0950_t15_a000_x0y1 | predicted_region | 7944 | 0.0485 | 0.0285 | 0.1679 | yes |
| b_z0950_t15_a090_x0y1 | predicted_region | 9377 | 0.0482 | 0.0217 | 0.1774 | yes |
| b_z0950_t15_a180_x0y1 | predicted_region | 9461 | 0.0504 | 0.0147 | -0.0763 | yes |
| b_z0950_t15_a270_x0y1 | predicted_region | 9853 | 0.0470 | 0.0263 | 0.0525 | yes |
| b_z0950_t30_a000_x0y1 | predicted_region | 1484 | 0.0474 | 0.0228 | -0.1224 | yes |
| b_z0950_t30_a090_x0y1 | predicted_region | 2902 | 0.0486 | 0.0408 | 0.4242 | yes |
| b_z0950_t30_a180_x0y1 | predicted_region | 553 | 0.0444 | 0.0144 | -0.0439 | yes |
| b_z0950_t30_a270_x0y1 | predicted_region | 935 | 0.0501 | 0.0537 | -0.3856 | yes |
| b_z0950_t00_a000_x0y2 | predicted_region | 10972 | 0.0483 | 0.0187 | 0.0851 | yes |
| b_z0950_t15_a000_x0y2 | predicted_region | 8173 | 0.0481 | 0.0249 | 0.1474 | yes |
| b_z0950_t15_a090_x0y2 | predicted_region | 9081 | 0.0515 | 0.0206 | -0.1666 | yes |
| b_z0950_t15_a180_x0y2 | predicted_region | 6571 | 0.0498 | 0.0152 | -0.1043 | yes |
| b_z0950_t15_a270_x0y2 | predicted_region | 8940 | 0.0507 | 0.0149 | 0.0319 | yes |
| b_z0950_t30_a000_x0y2 | predicted_region | 660 | 0.0505 | 0.0506 | -0.0988 | yes |
| b_z0950_t30_a090_x0y2 | predicted_region | 5087 | 0.0534 | 0.0108 | -0.1685 | yes |
| b_z0950_t30_a180_x0y2 | predicted_region | 4060 | 0.0510 | 0.0059 | -0.0106 | yes |
| b_z0950_t30_a270_x0y2 | predicted_region | 3837 | 0.0504 | 0.0229 | 0.1460 | yes |
| b_z0950_t00_a000_x1y2 | predicted_region | 10587 | 0.0465 | 0.0260 | -0.0435 | yes |
| b_z0950_t15_a000_x1y2 | predicted_region | 10128 | 0.0492 | 0.0244 | 0.1404 | yes |
| b_z0950_t15_a090_x1y2 | predicted_region | 8897 | 0.0486 | 0.0225 | 0.1617 | yes |
| b_z0950_t15_a180_x1y2 | predicted_region | 9317 | 0.0488 | 0.0092 | -0.0095 | yes |
| b_z0950_t15_a270_x1y2 | predicted_region | 8862 | 0.0497 | 0.0236 | 0.0625 | yes |
| b_z0950_t30_a000_x1y2 | predicted_region | 2114 | 0.0490 | 0.0486 | -0.3235 | yes |
| b_z0950_t30_a090_x1y2 | predicted_region | 3212 | 0.0435 | 0.0204 | 0.2465 | yes |
| b_z0950_t30_a180_x1y2 | predicted_region | 1622 | 0.0456 | 0.0378 | 0.3180 | yes |
| b_z0950_t30_a270_x1y2 | predicted_region | 3410 | 0.0467 | 0.0224 | -0.0369 | yes |
| b_z0950_t00_a000_x2y2 | predicted_region | 10971 | 0.0500 | 0.0203 | -0.0141 | yes |
| b_z0950_t15_a000_x2y2 | predicted_region | 9263 | 0.0510 | 0.0233 | 0.1375 | yes |
| b_z0950_t15_a090_x2y2 | predicted_region | 8825 | 0.0510 | 0.0199 | 0.1445 | yes |
| b_z0950_t15_a180_x2y2 | predicted_region | 9042 | 0.0476 | 0.0235 | -0.0388 | yes |
| b_z0950_t15_a270_x2y2 | predicted_region | 8856 | 0.0493 | 0.0246 | 0.1128 | yes |
| b_z0950_t30_a000_x2y2 | predicted_region | 706 | 0.0480 | 0.0444 | -0.2623 | yes |
| b_z0950_t30_a090_x2y2 | predicted_region | 5003 | 0.0486 | 0.0133 | -0.1960 | yes |
| b_z0950_t30_a180_x2y2 | predicted_region | 886 | 0.0450 | 0.0651 | -0.0787 | yes |
| b_z0950_t30_a270_x2y2 | predicted_region | 511 | 0.0452 | 0.0658 | 0.0971 | yes |

### Errors against the known truth

| pass | rotation error (deg) | translation error (mm) | scale | scale error |
|---|---|---|---|---|
| pass1 | 0.0037 | 0.0318 | 0.999965 | -0.000035 |
| pass2 | 0.0038 | 0.0320 | 0.999966 | -0.000034 |

## Comparison of two sessions

### Model: rigid

| quantity | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0286 | 0.0163 |
| RMS offset residual (mm) | 0.1587 | 0.0762 |
| poses used | 235 | 235 |

Session B minimizing backlash has the smaller residual RMS of the two (235 poses in common).
Relative transform between the two solutions: rotation 0.0273 deg, translation 0.2785 mm, scale ratio 1.000000.

### Model: similarity

| quantity | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0286 | 0.0163 |
| RMS offset residual (mm) | 0.1586 | 0.0762 |
| poses used | 235 | 235 |

Session B minimizing backlash has the smaller residual RMS of the two (235 poses in common).
Relative transform between the two solutions: rotation 0.0273 deg, translation 0.2720 mm, scale ratio 1.000032.

## Figures

- [normal_residual_field.png](analysis_A/figures/rigid/normal_residual_field.png)
- [offset_residual_field.png](analysis_A/figures/rigid/offset_residual_field.png)
- [pixel_residuals_b_z0550_t15_a180_x2y2.png](analysis_A/figures/rigid/pixel_residuals_b_z0550_t15_a180_x2y2.png)
- [pixel_residuals_b_z0750_t30_a270_x1y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0750_t30_a270_x1y1.png)
- [pixel_residuals_b_z0950_t30_a000_x1y2.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a000_x1y2.png)
- [pixel_residuals_b_z0950_t30_a000_x2y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a000_x2y0.png)
- [pixel_residuals_b_z0950_t30_a090_x0y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a090_x0y1.png)
- [pixel_residuals_b_z0950_t30_a180_x0y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a180_x0y0.png)
- [pixel_residuals_b_z0950_t30_a270_x0y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a270_x0y1.png)
- [pixel_residuals_b_z0950_t30_a270_x1y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a270_x1y0.png)
- [pose_residuals.png](analysis_A/figures/rigid/pose_residuals.png)
- [normal_residual_field.png](analysis_A/figures/similarity/normal_residual_field.png)
- [offset_residual_field.png](analysis_A/figures/similarity/offset_residual_field.png)
- [pixel_residuals_b_z0550_t15_a180_x0y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0550_t15_a180_x0y0.png)
- [pixel_residuals_b_z0550_t30_a180_x1y2.png](analysis_A/figures/similarity/pixel_residuals_b_z0550_t30_a180_x1y2.png)
- [pixel_residuals_b_z0950_t30_a000_x1y2.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a000_x1y2.png)
- [pixel_residuals_b_z0950_t30_a000_x2y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a000_x2y0.png)
- [pixel_residuals_b_z0950_t30_a090_x0y1.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a090_x0y1.png)
- [pixel_residuals_b_z0950_t30_a180_x0y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a180_x0y0.png)
- [pixel_residuals_b_z0950_t30_a270_x0y1.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a270_x0y1.png)
- [pixel_residuals_b_z0950_t30_a270_x1y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a270_x1y0.png)
- [pose_residuals.png](analysis_A/figures/similarity/pose_residuals.png)
- [comparison.png](comparison/comparison.png)
