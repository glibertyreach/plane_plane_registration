# Registration report

Manifest: /tmp/claude-0/-home-user/fd747cf1-d9f9-5dd3-84db-c067a012fbe6/scratchpad/rehearsal3/captures_A/manifest.csv

Rough transform for the first pass: /tmp/claude-0/-home-user/fd747cf1-d9f9-5dd3-84db-c067a012fbe6/scratchpad/rehearsal3/boot.json.
Outlier rejection rounds: 0.
Pose plan (held-out poses): /tmp/claude-0/-home-user/fd747cf1-d9f9-5dd3-84db-c067a012fbe6/scratchpad/rehearsal3/plan/poses.csv; 0 of 235 plan poses were not captured, 0 manifest poses are not in the plan.

## Model: rigid

Verdict: ACCEPTED. Status Success: registration succeeded. Final pass: pass2; 179 of 235 poses used.

### Transform (sensor to base)

```
  1.000000  -0.000080   0.000012 799.990960
 -0.000080  -1.000000  -0.000006   0.010251
  0.000012   0.000006  -1.000000 1200.004813
  0.000000   0.000000   0.000000   1.000000
```

Rotation vector 179.9997, -0.0072, 0.0011 deg; translation 799.9910, 0.0103, 1200.0048 mm; scale 1.000000.

### Residuals against the acceptance thresholds

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0242 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.0646 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1249 | 0.5000 | within the limit |
| maximum offset residual (mm) | 0.4209 | 1.0000 | within the limit |

### Held-out poses

45 held-out poses were left out of the solve and evaluated against its transform.
Held-out poses that could not be evaluated (segmentation failed or board image too small): b_z0950_t30_a180_x0y0, b_z0950_t30_a270_x1y0.

Verdict on the held-out poses: within the limit.

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0237 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.0358 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1321 | 0.5000 | within the limit |
| maximum offset residual (mm) | 0.2986 | 1.0000 | within the limit |

### Spreads of the pose set

| quantity | value | minimum | result |
|---|---|---|---|
| normal spread | 0.2723 | 0.0500 | at or above the minimum |

### Passes

| pass | status (candidate regions from) | poses used | RMS normal (deg) | RMS offset (mm) |
|---|---|---|---|---|
| pass1 | Success (sensor_in_base) | 179/235 | 0.0242 | 0.1252 |
| pass2 | Success (pass1_transform) | 179/235 | 0.0242 | 0.1249 |

### Segmentation

Segmentation methods: predicted_region 235.
Mask pixels per pose: minimum 491, median 10973, maximum 35048.
11 poses left out as too small (fewer than 1000 pixels): b_z0750_t30_a000_x2y2, b_z0950_t30_a180_x0y0, b_z0950_t30_a270_x1y0, b_z0950_t30_a000_x2y0, b_z0950_t30_a180_x2y1, b_z0950_t30_a180_x0y1, b_z0950_t30_a270_x0y1, b_z0950_t30_a000_x0y2, b_z0950_t30_a000_x2y2, b_z0950_t30_a180_x2y2, b_z0950_t30_a270_x2y2.
Poses whose segmentation failed: none.
Poses rejected as outliers: none.

### Residual of every pose

| pose | method | pixels | plane RMS (mm) | normal (deg) | offset (mm) | used | left out |
|---|---|---|---|---|---|---|---|
| b_z0550_t00_a000_x0y0 | predicted_region | 35048 | 0.0170 | 0.0221 | 0.0846 | no | held out |
| b_z0550_t15_a000_x0y0 | predicted_region | 31628 | 0.0169 | 0.0304 | -0.1078 | no | held out |
| b_z0550_t15_a090_x0y0 | predicted_region | 31361 | 0.0171 | 0.0243 | -0.1560 | yes |  |
| b_z0550_t15_a180_x0y0 | predicted_region | 31371 | 0.0166 | 0.0190 | 0.0612 | no | held out |
| b_z0550_t15_a270_x0y0 | predicted_region | 30642 | 0.0167 | 0.0235 | 0.0148 | no | held out |
| b_z0550_t30_a180_x0y0 | predicted_region | 12488 | 0.0164 | 0.0127 | -0.0194 | yes |  |
| b_z0550_t30_a270_x0y0 | predicted_region | 13131 | 0.0163 | 0.0312 | 0.0782 | no | held out |
| b_z0550_t00_a000_x1y0 | predicted_region | 33850 | 0.0160 | 0.0257 | -0.0689 | no | held out |
| b_z0550_t15_a000_x1y0 | predicted_region | 28995 | 0.0168 | 0.0319 | -0.1018 | yes |  |
| b_z0550_t15_a090_x1y0 | predicted_region | 29916 | 0.0162 | 0.0220 | 0.1428 | yes |  |
| b_z0550_t15_a180_x1y0 | predicted_region | 28975 | 0.0165 | 0.0229 | -0.0284 | yes |  |
| b_z0550_t15_a270_x1y0 | predicted_region | 31107 | 0.0165 | 0.0295 | 0.0118 | yes |  |
| b_z0550_t30_a000_x1y0 | predicted_region | 14283 | 0.0160 | 0.0275 | 0.1178 | yes |  |
| b_z0550_t30_a090_x1y0 | predicted_region | 12911 | 0.0154 | 0.0232 | 0.2101 | yes |  |
| b_z0550_t30_a180_x1y0 | predicted_region | 12590 | 0.0166 | 0.0155 | 0.0097 | no | held out |
| b_z0550_t30_a270_x1y0 | predicted_region | 13901 | 0.0165 | 0.0299 | 0.0858 | yes |  |
| b_z0550_t00_a000_x2y0 | predicted_region | 34818 | 0.0162 | 0.0289 | -0.0495 | yes |  |
| b_z0550_t15_a000_x2y0 | predicted_region | 30453 | 0.0168 | 0.0317 | -0.0907 | yes |  |
| b_z0550_t15_a090_x2y0 | predicted_region | 32652 | 0.0168 | 0.0264 | -0.1291 | no | held out |
| b_z0550_t15_a180_x2y0 | predicted_region | 31421 | 0.0165 | 0.0208 | 0.0227 | no | held out |
| b_z0550_t15_a270_x2y0 | predicted_region | 31079 | 0.0166 | 0.0262 | -0.0225 | yes |  |
| b_z0550_t30_a090_x2y0 | predicted_region | 15094 | 0.0166 | 0.0229 | 0.2043 | yes |  |
| b_z0550_t30_a180_x2y0 | predicted_region | 11962 | 0.0167 | 0.0204 | 0.0418 | yes |  |
| b_z0550_t00_a000_x2y1 | predicted_region | 34177 | 0.0164 | 0.0265 | -0.0435 | yes |  |
| b_z0550_t15_a000_x2y1 | predicted_region | 29601 | 0.0167 | 0.0292 | -0.0882 | yes |  |
| b_z0550_t15_a090_x2y1 | predicted_region | 29446 | 0.0164 | 0.0208 | 0.1223 | yes |  |
| b_z0550_t15_a180_x2y1 | predicted_region | 31151 | 0.0172 | 0.0200 | 0.0160 | yes |  |
| b_z0550_t15_a270_x2y1 | predicted_region | 32249 | 0.0162 | 0.0288 | 0.0308 | yes |  |
| b_z0550_t30_a000_x2y1 | predicted_region | 12583 | 0.0165 | 0.0324 | -0.1197 | no | held out |
| b_z0550_t30_a090_x2y1 | predicted_region | 13308 | 0.0168 | 0.0196 | -0.1731 | yes |  |
| b_z0550_t30_a180_x2y1 | predicted_region | 12353 | 0.0162 | 0.0171 | -0.0363 | yes |  |
| b_z0550_t30_a270_x2y1 | predicted_region | 14801 | 0.0156 | 0.0271 | -0.0930 | yes |  |
| b_z0550_t00_a000_x1y1 | predicted_region | 33227 | 0.0157 | 0.0234 | -0.0617 | yes |  |
| b_z0550_t15_a000_x1y1 | predicted_region | 31492 | 0.0164 | 0.0285 | -0.0986 | yes |  |
| b_z0550_t15_a090_x1y1 | predicted_region | 27607 | 0.0163 | 0.0207 | 0.1341 | no | held out |
| b_z0550_t15_a180_x1y1 | predicted_region | 31657 | 0.0162 | 0.0171 | 0.0394 | yes |  |
| b_z0550_t15_a270_x1y1 | predicted_region | 29994 | 0.0166 | 0.0244 | 0.0006 | yes |  |
| b_z0550_t30_a000_x1y1 | predicted_region | 5018 | 0.0154 | 0.0333 | -0.1382 | yes |  |
| b_z0550_t30_a090_x1y1 | predicted_region | 14964 | 0.0157 | 0.0179 | -0.1787 | no | held out |
| b_z0550_t30_a180_x1y1 | predicted_region | 11547 | 0.0160 | 0.0127 | 0.0171 | yes |  |
| b_z0550_t30_a270_x1y1 | predicted_region | 13345 | 0.0149 | 0.0239 | -0.0571 | yes |  |
| b_z0550_t00_a000_x0y1 | predicted_region | 34635 | 0.0165 | 0.0201 | 0.0823 | yes |  |
| b_z0550_t15_a000_x0y1 | predicted_region | 29701 | 0.0163 | 0.0234 | 0.0979 | yes |  |
| b_z0550_t15_a090_x0y1 | predicted_region | 32424 | 0.0161 | 0.0217 | -0.1442 | yes |  |
| b_z0550_t15_a180_x0y1 | predicted_region | 31536 | 0.0168 | 0.0148 | 0.0632 | yes |  |
| b_z0550_t15_a270_x0y1 | predicted_region | 30267 | 0.0165 | 0.0232 | -0.0106 | yes |  |
| b_z0550_t30_a000_x0y1 | predicted_region | 13516 | 0.0163 | 0.0259 | 0.1100 | yes |  |
| b_z0550_t30_a090_x0y1 | predicted_region | 15914 | 0.0156 | 0.0230 | -0.1987 | yes |  |
| b_z0550_t30_a180_x0y1 | predicted_region | 14952 | 0.0170 | 0.0083 | -0.0108 | yes |  |
| b_z0550_t30_a270_x0y1 | predicted_region | 13101 | 0.0165 | 0.0258 | 0.0598 | yes |  |
| b_z0550_t00_a000_x0y2 | predicted_region | 35031 | 0.0175 | 0.0162 | -0.0697 | yes |  |
| b_z0550_t15_a000_x0y2 | predicted_region | 33019 | 0.0168 | 0.0218 | 0.0978 | no | held out |
| b_z0550_t15_a090_x0y2 | predicted_region | 33477 | 0.0166 | 0.0166 | 0.1272 | yes |  |
| b_z0550_t15_a180_x0y2 | predicted_region | 30385 | 0.0172 | 0.0117 | 0.0539 | yes |  |
| b_z0550_t15_a270_x0y2 | predicted_region | 30471 | 0.0163 | 0.0188 | -0.0127 | yes |  |
| b_z0550_t30_a000_x0y2 | predicted_region | 13474 | 0.0170 | 0.0289 | -0.0894 | yes |  |
| b_z0550_t30_a270_x0y2 | predicted_region | 13361 | 0.0162 | 0.0225 | 0.0475 | yes |  |
| b_z0550_t00_a000_x1y2 | predicted_region | 34062 | 0.0160 | 0.0188 | -0.0541 | no | held out |
| b_z0550_t15_a000_x1y2 | predicted_region | 31084 | 0.0166 | 0.0233 | 0.0979 | yes |  |
| b_z0550_t15_a090_x1y2 | predicted_region | 28356 | 0.0164 | 0.0164 | 0.1196 | no | held out |
| b_z0550_t15_a180_x1y2 | predicted_region | 31192 | 0.0166 | 0.0136 | -0.0126 | yes |  |
| b_z0550_t15_a270_x1y2 | predicted_region | 31405 | 0.0159 | 0.0217 | 0.0063 | yes |  |
| b_z0550_t30_a000_x1y2 | predicted_region | 12558 | 0.0165 | 0.0253 | 0.1235 | yes |  |
| b_z0550_t30_a090_x1y2 | predicted_region | 14154 | 0.0163 | 0.0153 | -0.1557 | yes |  |
| b_z0550_t30_a180_x1y2 | predicted_region | 13556 | 0.0164 | 0.0117 | 0.0325 | yes |  |
| b_z0550_t30_a270_x1y2 | predicted_region | 13281 | 0.0154 | 0.0288 | 0.0762 | yes |  |
| b_z0550_t00_a000_x2y2 | predicted_region | 35043 | 0.0167 | 0.0210 | 0.0515 | yes |  |
| b_z0550_t15_a000_x2y2 | predicted_region | 33405 | 0.0163 | 0.0263 | -0.0829 | no | held out |
| b_z0550_t15_a090_x2y2 | predicted_region | 30287 | 0.0174 | 0.0170 | -0.0968 | yes |  |
| b_z0550_t15_a180_x2y2 | predicted_region | 32847 | 0.0167 | 0.0193 | 0.0200 | yes |  |
| b_z0550_t15_a270_x2y2 | predicted_region | 33072 | 0.0167 | 0.0269 | 0.0272 | yes |  |
| b_z0550_t30_a000_x2y2 | predicted_region | 12709 | 0.0170 | 0.0256 | 0.1333 | yes |  |
| b_z0550_t30_a090_x2y2 | predicted_region | 13518 | 0.0165 | 0.0181 | 0.1640 | yes |  |
| b_z0750_t00_a000_x0y0 | predicted_region | 18207 | 0.0314 | 0.0254 | -0.0856 | yes |  |
| b_z0750_t15_a000_x0y0 | predicted_region | 14734 | 0.0307 | 0.0302 | -0.1200 | yes |  |
| b_z0750_t15_a090_x0y0 | predicted_region | 15174 | 0.0301 | 0.0267 | -0.1891 | yes |  |
| b_z0750_t15_a180_x0y0 | predicted_region | 14458 | 0.0312 | 0.0202 | -0.0399 | yes |  |
| b_z0750_t15_a270_x0y0 | predicted_region | 16034 | 0.0314 | 0.0237 | -0.0053 | yes |  |
| b_z0750_t30_a000_x0y0 | predicted_region | 4655 | 0.0323 | 0.0239 | 0.1690 | yes |  |
| b_z0750_t30_a090_x0y0 | predicted_region | 6357 | 0.0283 | 0.0290 | -0.2834 | yes |  |
| b_z0750_t30_a180_x0y0 | predicted_region | 7397 | 0.0334 | 0.0121 | -0.0033 | yes |  |
| b_z0750_t30_a270_x0y0 | predicted_region | 8169 | 0.0319 | 0.0229 | -0.0951 | yes |  |
| b_z0750_t00_a000_x1y0 | predicted_region | 17737 | 0.0300 | 0.0207 | 0.0731 | yes |  |
| b_z0750_t15_a000_x1y0 | predicted_region | 12600 | 0.0297 | 0.0289 | -0.0967 | no | held out |
| b_z0750_t15_a090_x1y0 | predicted_region | 15410 | 0.0294 | 0.0246 | -0.1657 | yes |  |
| b_z0750_t15_a180_x1y0 | predicted_region | 16058 | 0.0307 | 0.0233 | 0.0323 | yes |  |
| b_z0750_t15_a270_x1y0 | predicted_region | 15782 | 0.0295 | 0.0227 | -0.0243 | yes |  |
| b_z0750_t30_a000_x1y0 | predicted_region | 3852 | 0.0287 | 0.0305 | -0.1175 | yes |  |
| b_z0750_t30_a090_x1y0 | predicted_region | 7027 | 0.0278 | 0.0242 | -0.2450 | yes |  |
| b_z0750_t30_a180_x1y0 | predicted_region | 4288 | 0.0314 | 0.0086 | 0.0293 | yes |  |
| b_z0750_t30_a270_x1y0 | predicted_region | 7071 | 0.0290 | 0.0298 | -0.1500 | yes |  |
| b_z0750_t00_a000_x2y0 | predicted_region | 18208 | 0.0300 | 0.0249 | -0.0517 | yes |  |
| b_z0750_t15_a000_x2y0 | predicted_region | 16116 | 0.0310 | 0.0267 | 0.0985 | no | held out |
| b_z0750_t15_a090_x2y0 | predicted_region | 15118 | 0.0310 | 0.0239 | 0.1563 | yes |  |
| b_z0750_t15_a180_x2y0 | predicted_region | 15536 | 0.0302 | 0.0234 | -0.0024 | yes |  |
| b_z0750_t15_a270_x2y0 | predicted_region | 15195 | 0.0313 | 0.0272 | -0.0405 | yes |  |
| b_z0750_t30_a000_x2y0 | predicted_region | 5562 | 0.0336 | 0.0278 | 0.1309 | no | held out |
| b_z0750_t30_a090_x2y0 | predicted_region | 7212 | 0.0298 | 0.0273 | -0.2565 | no | held out |
| b_z0750_t30_a180_x2y0 | predicted_region | 4303 | 0.0327 | 0.0235 | 0.1027 | yes |  |
| b_z0750_t30_a270_x2y0 | predicted_region | 7449 | 0.0304 | 0.0312 | -0.1521 | yes |  |
| b_z0750_t00_a000_x2y1 | predicted_region | 18011 | 0.0298 | 0.0266 | -0.0447 | yes |  |
| b_z0750_t15_a000_x2y1 | predicted_region | 16215 | 0.0321 | 0.0271 | -0.1032 | no | held out |
| b_z0750_t15_a090_x2y1 | predicted_region | 15535 | 0.0304 | 0.0226 | -0.1363 | yes |  |
| b_z0750_t15_a180_x2y1 | predicted_region | 15101 | 0.0313 | 0.0216 | 0.0262 | yes |  |
| b_z0750_t15_a270_x2y1 | predicted_region | 16277 | 0.0305 | 0.0280 | 0.0536 | yes |  |
| b_z0750_t30_a000_x2y1 | predicted_region | 5779 | 0.0307 | 0.0248 | 0.1699 | yes |  |
| b_z0750_t30_a090_x2y1 | predicted_region | 4900 | 0.0314 | 0.0229 | 0.2371 | yes |  |
| b_z0750_t30_a180_x2y1 | predicted_region | 3832 | 0.0316 | 0.0096 | -0.0287 | yes |  |
| b_z0750_t30_a270_x2y1 | predicted_region | 4184 | 0.0301 | 0.0382 | 0.1617 | yes |  |
| b_z0750_t00_a000_x1y1 | predicted_region | 17427 | 0.0301 | 0.0222 | -0.0599 | no | held out |
| b_z0750_t15_a000_x1y1 | predicted_region | 14537 | 0.0304 | 0.0277 | 0.1158 | yes |  |
| b_z0750_t15_a090_x1y1 | predicted_region | 15487 | 0.0316 | 0.0197 | -0.1437 | no | held out |
| b_z0750_t15_a180_x1y1 | predicted_region | 15232 | 0.0296 | 0.0187 | -0.0028 | yes |  |
| b_z0750_t15_a270_x1y1 | predicted_region | 14744 | 0.0293 | 0.0228 | -0.0049 | yes |  |
| b_z0750_t30_a000_x1y1 | predicted_region | 7515 | 0.0328 | 0.0346 | -0.1193 | yes |  |
| b_z0750_t30_a090_x1y1 | predicted_region | 3737 | 0.0276 | 0.0167 | 0.1331 | no | held out |
| b_z0750_t30_a180_x1y1 | predicted_region | 1713 | 0.0295 | 0.0079 | -0.0218 | yes |  |
| b_z0750_t30_a270_x1y1 | predicted_region | 8490 | 0.0284 | 0.0261 | -0.0796 | yes |  |
| b_z0750_t00_a000_x0y1 | predicted_region | 18015 | 0.0309 | 0.0228 | 0.0818 | yes |  |
| b_z0750_t15_a000_x0y1 | predicted_region | 15891 | 0.0293 | 0.0258 | -0.1053 | no | held out |
| b_z0750_t15_a090_x0y1 | predicted_region | 16279 | 0.0300 | 0.0196 | 0.1580 | yes |  |
| b_z0750_t15_a180_x0y1 | predicted_region | 16443 | 0.0307 | 0.0146 | -0.0383 | no | held out |
| b_z0750_t15_a270_x0y1 | predicted_region | 15221 | 0.0310 | 0.0231 | 0.0028 | no | held out |
| b_z0750_t30_a000_x0y1 | predicted_region | 7027 | 0.0315 | 0.0288 | -0.1064 | no | held out |
| b_z0750_t30_a090_x0y1 | predicted_region | 2679 | 0.0293 | 0.0211 | -0.2288 | yes |  |
| b_z0750_t30_a180_x0y1 | predicted_region | 3183 | 0.0333 | 0.0022 | -0.0347 | yes |  |
| b_z0750_t30_a270_x0y1 | predicted_region | 5752 | 0.0297 | 0.0246 | -0.1020 | yes |  |
| b_z0750_t00_a000_x0y2 | predicted_region | 18211 | 0.0308 | 0.0174 | -0.0711 | yes |  |
| b_z0750_t15_a000_x0y2 | predicted_region | 17206 | 0.0301 | 0.0235 | 0.0925 | yes |  |
| b_z0750_t15_a090_x0y2 | predicted_region | 15831 | 0.0322 | 0.0174 | -0.1448 | no | held out |
| b_z0750_t15_a180_x0y2 | predicted_region | 15766 | 0.0305 | 0.0081 | -0.0348 | yes |  |
| b_z0750_t15_a270_x0y2 | predicted_region | 15791 | 0.0308 | 0.0174 | 0.0130 | yes |  |
| b_z0750_t30_a000_x0y2 | predicted_region | 4735 | 0.0316 | 0.0248 | 0.1128 | yes |  |
| b_z0750_t30_a090_x0y2 | predicted_region | 7434 | 0.0321 | 0.0244 | -0.2442 | yes |  |
| b_z0750_t30_a180_x0y2 | predicted_region | 4710 | 0.0300 | 0.0137 | 0.0586 | yes |  |
| b_z0750_t30_a270_x0y2 | predicted_region | 6137 | 0.0318 | 0.0241 | 0.0862 | yes |  |
| b_z0750_t00_a000_x1y2 | predicted_region | 17735 | 0.0312 | 0.0163 | -0.0546 | yes |  |
| b_z0750_t15_a000_x1y2 | predicted_region | 15417 | 0.0300 | 0.0258 | -0.1096 | yes |  |
| b_z0750_t15_a090_x1y2 | predicted_region | 16426 | 0.0298 | 0.0133 | -0.1186 | yes |  |
| b_z0750_t15_a180_x1y2 | predicted_region | 15599 | 0.0309 | 0.0140 | 0.0020 | yes |  |
| b_z0750_t15_a270_x1y2 | predicted_region | 15273 | 0.0295 | 0.0240 | 0.0189 | yes |  |
| b_z0750_t30_a000_x1y2 | predicted_region | 5179 | 0.0311 | 0.0245 | 0.1467 | no | held out |
| b_z0750_t30_a090_x1y2 | predicted_region | 6772 | 0.0296 | 0.0190 | 0.2065 | yes |  |
| b_z0750_t30_a180_x1y2 | predicted_region | 7127 | 0.0311 | 0.0098 | 0.0375 | yes |  |
| b_z0750_t30_a270_x1y2 | predicted_region | 4255 | 0.0289 | 0.0283 | -0.0833 | yes |  |
| b_z0750_t00_a000_x2y2 | predicted_region | 18209 | 0.0314 | 0.0197 | -0.0341 | yes |  |
| b_z0750_t15_a000_x2y2 | predicted_region | 16074 | 0.0315 | 0.0305 | -0.1036 | yes |  |
| b_z0750_t15_a090_x2y2 | predicted_region | 16872 | 0.0306 | 0.0185 | -0.1128 | yes |  |
| b_z0750_t15_a180_x2y2 | predicted_region | 15470 | 0.0308 | 0.0196 | -0.0226 | no | held out |
| b_z0750_t15_a270_x2y2 | predicted_region | 15791 | 0.0308 | 0.0234 | -0.0261 | yes |  |
| b_z0750_t30_a000_x2y2 | predicted_region | 766 | 0.0319 | 0.0308 | 0.1318 | no | too small |
| b_z0750_t30_a090_x2y2 | predicted_region | 6984 | 0.0314 | 0.0151 | -0.1728 | yes |  |
| b_z0750_t30_a180_x2y2 | predicted_region | 8159 | 0.0307 | 0.0154 | -0.0856 | yes |  |
| b_z0750_t30_a270_x2y2 | predicted_region | 5589 | 0.0310 | 0.0283 | -0.1136 | yes |  |
| b_z0950_t00_a000_x0y0 | predicted_region | 10862 | 0.0499 | 0.0261 | 0.0848 | yes |  |
| b_z0950_t15_a000_x0y0 | predicted_region | 5794 | 0.0487 | 0.0292 | -0.0835 | yes |  |
| b_z0950_t15_a090_x0y0 | predicted_region | 10171 | 0.0487 | 0.0297 | 0.2174 | no | held out |
| b_z0950_t15_a180_x0y0 | predicted_region | 8366 | 0.0473 | 0.0282 | 0.0045 | yes |  |
| b_z0950_t15_a270_x0y0 | predicted_region | 8890 | 0.0476 | 0.0286 | -0.0246 | yes |  |
| b_z0950_t30_a000_x0y0 | predicted_region | 4366 | 0.0524 | 0.0358 | 0.2307 | no | held out |
| b_z0950_t30_a090_x0y0 | predicted_region | 5188 | 0.0513 | 0.0232 | 0.2986 | no | held out |
| b_z0950_t30_a180_x0y0 | predicted_region | 518 | 0.0454 | 0.0699 | -0.4773 | no | too small |
| b_z0950_t30_a270_x0y0 | predicted_region | 1109 | 0.0471 | 0.0646 | -0.2938 | yes |  |
| b_z0950_t00_a000_x1y0 | predicted_region | 10668 | 0.0462 | 0.0263 | 0.0760 | yes |  |
| b_z0950_t15_a000_x1y0 | predicted_region | 6715 | 0.0482 | 0.0276 | 0.1079 | yes |  |
| b_z0950_t15_a090_x1y0 | predicted_region | 8732 | 0.0489 | 0.0290 | 0.2167 | no | held out |
| b_z0950_t15_a180_x1y0 | predicted_region | 8918 | 0.0470 | 0.0162 | -0.0582 | yes |  |
| b_z0950_t15_a270_x1y0 | predicted_region | 9400 | 0.0459 | 0.0280 | 0.0498 | yes |  |
| b_z0950_t30_a000_x1y0 | predicted_region | 1947 | 0.0513 | 0.0150 | -0.0071 | yes |  |
| b_z0950_t30_a090_x1y0 | predicted_region | 4623 | 0.0468 | 0.0253 | 0.3026 | yes |  |
| b_z0950_t30_a180_x1y0 | predicted_region | 1985 | 0.0517 | 0.0118 | 0.0114 | yes |  |
| b_z0950_t30_a270_x1y0 | predicted_region | 918 | 0.0516 | 0.0472 | 0.2970 | no | too small |
| b_z0950_t00_a000_x2y0 | predicted_region | 10831 | 0.0505 | 0.0271 | -0.0512 | yes |  |
| b_z0950_t15_a000_x2y0 | predicted_region | 10493 | 0.0506 | 0.0271 | 0.1287 | yes |  |
| b_z0950_t15_a090_x2y0 | predicted_region | 9785 | 0.0482 | 0.0325 | 0.1968 | yes |  |
| b_z0950_t15_a180_x2y0 | predicted_region | 9100 | 0.0495 | 0.0209 | -0.0111 | yes |  |
| b_z0950_t15_a270_x2y0 | predicted_region | 8445 | 0.0477 | 0.0288 | -0.0751 | yes |  |
| b_z0950_t30_a000_x2y0 | predicted_region | 725 | 0.0408 | 0.1670 | -1.2950 | no | too small |
| b_z0950_t30_a090_x2y0 | predicted_region | 1133 | 0.0509 | 0.0330 | -0.2514 | yes |  |
| b_z0950_t30_a180_x2y0 | predicted_region | 1349 | 0.0497 | 0.0094 | 0.0004 | yes |  |
| b_z0950_t30_a270_x2y0 | predicted_region | 1604 | 0.0469 | 0.0351 | -0.2405 | yes |  |
| b_z0950_t00_a000_x2y1 | predicted_region | 10839 | 0.0478 | 0.0224 | -0.0407 | yes |  |
| b_z0950_t15_a000_x2y1 | predicted_region | 8903 | 0.0478 | 0.0294 | -0.1213 | no | held out |
| b_z0950_t15_a090_x2y1 | predicted_region | 9112 | 0.0479 | 0.0207 | -0.1475 | no | held out |
| b_z0950_t15_a180_x2y1 | predicted_region | 8591 | 0.0467 | 0.0201 | 0.0287 | yes |  |
| b_z0950_t15_a270_x2y1 | predicted_region | 8593 | 0.0480 | 0.0234 | 0.0658 | no | held out |
| b_z0950_t30_a000_x2y1 | predicted_region | 1747 | 0.0451 | 0.0084 | 0.0771 | yes |  |
| b_z0950_t30_a090_x2y1 | predicted_region | 4612 | 0.0515 | 0.0192 | 0.2503 | yes |  |
| b_z0950_t30_a180_x2y1 | predicted_region | 491 | 0.0433 | 0.1006 | -0.2975 | no | too small |
| b_z0950_t30_a270_x2y1 | predicted_region | 2967 | 0.0498 | 0.0315 | 0.1778 | yes |  |
| b_z0950_t00_a000_x1y1 | predicted_region | 10701 | 0.0484 | 0.0252 | 0.0720 | yes |  |
| b_z0950_t15_a000_x1y1 | predicted_region | 8609 | 0.0488 | 0.0253 | 0.1264 | no | held out |
| b_z0950_t15_a090_x1y1 | predicted_region | 8735 | 0.0463 | 0.0223 | -0.1686 | yes |  |
| b_z0950_t15_a180_x1y1 | predicted_region | 8656 | 0.0476 | 0.0166 | 0.0204 | yes |  |
| b_z0950_t15_a270_x1y1 | predicted_region | 7098 | 0.0462 | 0.0245 | 0.0241 | no | held out |
| b_z0950_t30_a000_x1y1 | predicted_region | 2999 | 0.0501 | 0.0430 | 0.2725 | yes |  |
| b_z0950_t30_a090_x1y1 | predicted_region | 2548 | 0.0456 | 0.0245 | -0.2832 | yes |  |
| b_z0950_t30_a180_x1y1 | predicted_region | 1751 | 0.0428 | 0.0061 | -0.0134 | no | held out |
| b_z0950_t30_a270_x1y1 | predicted_region | 3270 | 0.0453 | 0.0252 | -0.1580 | yes |  |
| b_z0950_t00_a000_x0y1 | predicted_region | 10840 | 0.0469 | 0.0220 | 0.0850 | yes |  |
| b_z0950_t15_a000_x0y1 | predicted_region | 7942 | 0.0485 | 0.0271 | 0.1495 | yes |  |
| b_z0950_t15_a090_x0y1 | predicted_region | 9376 | 0.0482 | 0.0204 | 0.1684 | yes |  |
| b_z0950_t15_a180_x0y1 | predicted_region | 9459 | 0.0504 | 0.0159 | -0.0836 | yes |  |
| b_z0950_t15_a270_x0y1 | predicted_region | 9862 | 0.0469 | 0.0278 | 0.0358 | yes |  |
| b_z0950_t30_a000_x0y1 | predicted_region | 1484 | 0.0474 | 0.0247 | -0.1444 | no | held out |
| b_z0950_t30_a090_x0y1 | predicted_region | 2898 | 0.0485 | 0.0392 | 0.4209 | yes |  |
| b_z0950_t30_a180_x0y1 | predicted_region | 554 | 0.0445 | 0.0152 | -0.0519 | no | too small |
| b_z0950_t30_a270_x0y1 | predicted_region | 935 | 0.0501 | 0.0520 | -0.4046 | no | too small |
| b_z0950_t00_a000_x0y2 | predicted_region | 10973 | 0.0483 | 0.0175 | 0.0745 | yes |  |
| b_z0950_t15_a000_x0y2 | predicted_region | 8171 | 0.0481 | 0.0234 | 0.1327 | yes |  |
| b_z0950_t15_a090_x0y2 | predicted_region | 9082 | 0.0515 | 0.0221 | -0.1732 | yes |  |
| b_z0950_t15_a180_x0y2 | predicted_region | 6572 | 0.0498 | 0.0155 | -0.1100 | yes |  |
| b_z0950_t15_a270_x0y2 | predicted_region | 8931 | 0.0507 | 0.0134 | 0.0183 | yes |  |
| b_z0950_t30_a000_x0y2 | predicted_region | 660 | 0.0505 | 0.0525 | -0.1169 | no | too small |
| b_z0950_t30_a090_x0y2 | predicted_region | 5086 | 0.0534 | 0.0123 | -0.1709 | yes |  |
| b_z0950_t30_a180_x0y2 | predicted_region | 4062 | 0.0509 | 0.0054 | -0.0108 | no | held out |
| b_z0950_t30_a270_x0y2 | predicted_region | 3837 | 0.0504 | 0.0243 | 0.1296 | yes |  |
| b_z0950_t00_a000_x1y2 | predicted_region | 10586 | 0.0465 | 0.0272 | -0.0570 | yes |  |
| b_z0950_t15_a000_x1y2 | predicted_region | 10128 | 0.0492 | 0.0229 | 0.1229 | yes |  |
| b_z0950_t15_a090_x1y2 | predicted_region | 8895 | 0.0486 | 0.0214 | 0.1538 | yes |  |
| b_z0950_t15_a180_x1y2 | predicted_region | 9317 | 0.0488 | 0.0102 | -0.0177 | yes |  |
| b_z0950_t15_a270_x1y2 | predicted_region | 8855 | 0.0497 | 0.0250 | 0.0450 | yes |  |
| b_z0950_t30_a000_x1y2 | predicted_region | 2112 | 0.0490 | 0.0500 | -0.3416 | yes |  |
| b_z0950_t30_a090_x1y2 | predicted_region | 3213 | 0.0435 | 0.0191 | 0.2446 | no | held out |
| b_z0950_t30_a180_x1y2 | predicted_region | 1621 | 0.0456 | 0.0379 | 0.3152 | yes |  |
| b_z0950_t30_a270_x1y2 | predicted_region | 3408 | 0.0468 | 0.0210 | -0.0578 | yes |  |
| b_z0950_t00_a000_x2y2 | predicted_region | 10969 | 0.0500 | 0.0218 | -0.0332 | yes |  |
| b_z0950_t15_a000_x2y2 | predicted_region | 9263 | 0.0510 | 0.0216 | 0.1146 | yes |  |
| b_z0950_t15_a090_x2y2 | predicted_region | 8823 | 0.0509 | 0.0187 | 0.1317 | yes |  |
| b_z0950_t15_a180_x2y2 | predicted_region | 9039 | 0.0476 | 0.0225 | -0.0527 | yes |  |
| b_z0950_t15_a270_x2y2 | predicted_region | 8851 | 0.0493 | 0.0262 | 0.0885 | yes |  |
| b_z0950_t30_a000_x2y2 | predicted_region | 706 | 0.0480 | 0.0461 | -0.2878 | no | too small |
| b_z0950_t30_a090_x2y2 | predicted_region | 5002 | 0.0486 | 0.0147 | -0.2013 | no | held out |
| b_z0950_t30_a180_x2y2 | predicted_region | 886 | 0.0450 | 0.0656 | -0.0862 | no | too small |
| b_z0950_t30_a270_x2y2 | predicted_region | 510 | 0.0452 | 0.0640 | 0.0672 | no | too small |

### Errors against the known truth

| pass | rotation error (deg) | translation error (mm) | scale | scale error |
|---|---|---|---|---|
| pass1 | 0.0045 | 0.0140 | 1.000000 | 0.000000 |
| pass2 | 0.0046 | 0.0145 | 1.000000 | 0.000000 |

## Model: similarity

Verdict: ACCEPTED. Status Success: registration succeeded. Final pass: pass2; 179 of 235 poses used.

### Transform (sensor to base)

```
  1.000011  -0.000080   0.000012 799.991081
 -0.000080  -1.000011  -0.000006   0.010148
  0.000012   0.000006  -1.000011 1200.013688
  0.000000   0.000000   0.000000   1.000000
```

Rotation vector 179.9997, -0.0072, 0.0011 deg; translation 799.9911, 0.0101, 1200.0137 mm; scale 1.000011.

### Residuals against the acceptance thresholds

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0242 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.0646 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1249 | 0.5000 | within the limit |
| maximum offset residual (mm) | 0.4180 | 1.0000 | within the limit |

### Held-out poses

45 held-out poses were left out of the solve and evaluated against its transform.
Held-out poses that could not be evaluated (segmentation failed or board image too small): b_z0950_t30_a180_x0y0, b_z0950_t30_a270_x1y0.

Verdict on the held-out poses: within the limit.

| quantity | value | limit | result |
|---|---|---|---|
| RMS normal residual (deg) | 0.0237 | 0.5000 | within the limit |
| maximum normal residual (deg) | 0.0358 | 1.0000 | within the limit |
| RMS offset residual (mm) | 0.1317 | 0.5000 | within the limit |
| maximum offset residual (mm) | 0.2955 | 1.0000 | within the limit |

### Spreads of the pose set

| quantity | value | minimum | result |
|---|---|---|---|
| normal spread | 0.2723 | 0.0500 | at or above the minimum |
| similarity spread | 0.1281 | 0.0100 | at or above the minimum |

### Passes

| pass | status (candidate regions from) | poses used | RMS normal (deg) | RMS offset (mm) |
|---|---|---|---|---|
| pass1 | Success (sensor_in_base) | 179/235 | 0.0242 | 0.1252 |
| pass2 | Success (pass1_transform) | 179/235 | 0.0242 | 0.1249 |

### Segmentation

Segmentation methods: predicted_region 235.
Mask pixels per pose: minimum 491, median 10972, maximum 35047.
11 poses left out as too small (fewer than 1000 pixels): b_z0750_t30_a000_x2y2, b_z0950_t30_a180_x0y0, b_z0950_t30_a270_x1y0, b_z0950_t30_a000_x2y0, b_z0950_t30_a180_x2y1, b_z0950_t30_a180_x0y1, b_z0950_t30_a270_x0y1, b_z0950_t30_a000_x0y2, b_z0950_t30_a000_x2y2, b_z0950_t30_a180_x2y2, b_z0950_t30_a270_x2y2.
Poses whose segmentation failed: none.
Poses rejected as outliers: none.

### Residual of every pose

| pose | method | pixels | plane RMS (mm) | normal (deg) | offset (mm) | used | left out |
|---|---|---|---|---|---|---|---|
| b_z0550_t00_a000_x0y0 | predicted_region | 35047 | 0.0170 | 0.0221 | 0.0868 | no | held out |
| b_z0550_t15_a000_x0y0 | predicted_region | 31628 | 0.0169 | 0.0304 | -0.1062 | no | held out |
| b_z0550_t15_a090_x0y0 | predicted_region | 31360 | 0.0171 | 0.0243 | -0.1544 | yes |  |
| b_z0550_t15_a180_x0y0 | predicted_region | 31370 | 0.0166 | 0.0190 | 0.0636 | no | held out |
| b_z0550_t15_a270_x0y0 | predicted_region | 30643 | 0.0167 | 0.0235 | 0.0174 | no | held out |
| b_z0550_t30_a180_x0y0 | predicted_region | 12488 | 0.0164 | 0.0127 | -0.0168 | yes |  |
| b_z0550_t30_a270_x0y0 | predicted_region | 13131 | 0.0163 | 0.0312 | 0.0810 | no | held out |
| b_z0550_t00_a000_x1y0 | predicted_region | 33850 | 0.0160 | 0.0257 | -0.0664 | no | held out |
| b_z0550_t15_a000_x1y0 | predicted_region | 28994 | 0.0168 | 0.0319 | -0.0998 | yes |  |
| b_z0550_t15_a090_x1y0 | predicted_region | 29916 | 0.0162 | 0.0220 | 0.1452 | yes |  |
| b_z0550_t15_a180_x1y0 | predicted_region | 28975 | 0.0165 | 0.0229 | -0.0256 | yes |  |
| b_z0550_t15_a270_x1y0 | predicted_region | 31107 | 0.0165 | 0.0295 | 0.0142 | yes |  |
| b_z0550_t30_a000_x1y0 | predicted_region | 14283 | 0.0160 | 0.0275 | 0.1192 | yes |  |
| b_z0550_t30_a090_x1y0 | predicted_region | 12911 | 0.0154 | 0.0232 | 0.2123 | yes |  |
| b_z0550_t30_a180_x1y0 | predicted_region | 12590 | 0.0166 | 0.0155 | 0.0125 | no | held out |
| b_z0550_t30_a270_x1y0 | predicted_region | 13900 | 0.0165 | 0.0299 | 0.0879 | yes |  |
| b_z0550_t00_a000_x2y0 | predicted_region | 34817 | 0.0162 | 0.0289 | -0.0475 | yes |  |
| b_z0550_t15_a000_x2y0 | predicted_region | 30451 | 0.0168 | 0.0317 | -0.0890 | yes |  |
| b_z0550_t15_a090_x2y0 | predicted_region | 32650 | 0.0168 | 0.0264 | -0.1266 | no | held out |
| b_z0550_t15_a180_x2y0 | predicted_region | 31420 | 0.0165 | 0.0208 | 0.0250 | no | held out |
| b_z0550_t15_a270_x2y0 | predicted_region | 31078 | 0.0166 | 0.0262 | -0.0211 | yes |  |
| b_z0550_t30_a090_x2y0 | predicted_region | 15093 | 0.0166 | 0.0229 | 0.2071 | yes |  |
| b_z0550_t30_a180_x2y0 | predicted_region | 11962 | 0.0167 | 0.0204 | 0.0443 | yes |  |
| b_z0550_t00_a000_x2y1 | predicted_region | 34149 | 0.0164 | 0.0265 | -0.0412 | yes |  |
| b_z0550_t15_a000_x2y1 | predicted_region | 29600 | 0.0167 | 0.0293 | -0.0860 | yes |  |
| b_z0550_t15_a090_x2y1 | predicted_region | 29446 | 0.0164 | 0.0208 | 0.1251 | yes |  |
| b_z0550_t15_a180_x2y1 | predicted_region | 31150 | 0.0172 | 0.0200 | 0.0181 | yes |  |
| b_z0550_t15_a270_x2y1 | predicted_region | 32249 | 0.0162 | 0.0288 | 0.0324 | yes |  |
| b_z0550_t30_a000_x2y1 | predicted_region | 12581 | 0.0165 | 0.0324 | -0.1177 | no | held out |
| b_z0550_t30_a090_x2y1 | predicted_region | 13308 | 0.0168 | 0.0196 | -0.1701 | yes |  |
| b_z0550_t30_a180_x2y1 | predicted_region | 12352 | 0.0162 | 0.0171 | -0.0344 | yes |  |
| b_z0550_t30_a270_x2y1 | predicted_region | 14801 | 0.0156 | 0.0271 | -0.0921 | yes |  |
| b_z0550_t00_a000_x1y1 | predicted_region | 33227 | 0.0157 | 0.0234 | -0.0590 | yes |  |
| b_z0550_t15_a000_x1y1 | predicted_region | 31463 | 0.0164 | 0.0285 | -0.0959 | yes |  |
| b_z0550_t15_a090_x1y1 | predicted_region | 27606 | 0.0163 | 0.0207 | 0.1367 | no | held out |
| b_z0550_t15_a180_x1y1 | predicted_region | 31654 | 0.0162 | 0.0171 | 0.0419 | yes |  |
| b_z0550_t15_a270_x1y1 | predicted_region | 29992 | 0.0166 | 0.0244 | 0.0031 | yes |  |
| b_z0550_t30_a000_x1y1 | predicted_region | 5018 | 0.0154 | 0.0333 | -0.1358 | yes |  |
| b_z0550_t30_a090_x1y1 | predicted_region | 14964 | 0.0157 | 0.0179 | -0.1763 | no | held out |
| b_z0550_t30_a180_x1y1 | predicted_region | 11547 | 0.0160 | 0.0127 | 0.0194 | yes |  |
| b_z0550_t30_a270_x1y1 | predicted_region | 13344 | 0.0149 | 0.0239 | -0.0548 | yes |  |
| b_z0550_t00_a000_x0y1 | predicted_region | 34635 | 0.0165 | 0.0201 | 0.0846 | yes |  |
| b_z0550_t15_a000_x0y1 | predicted_region | 29702 | 0.0163 | 0.0234 | 0.1002 | yes |  |
| b_z0550_t15_a090_x0y1 | predicted_region | 32424 | 0.0161 | 0.0217 | -0.1425 | yes |  |
| b_z0550_t15_a180_x0y1 | predicted_region | 31535 | 0.0168 | 0.0148 | 0.0654 | yes |  |
| b_z0550_t15_a270_x0y1 | predicted_region | 30266 | 0.0165 | 0.0232 | -0.0079 | yes |  |
| b_z0550_t30_a000_x0y1 | predicted_region | 13516 | 0.0163 | 0.0259 | 0.1121 | yes |  |
| b_z0550_t30_a090_x0y1 | predicted_region | 15914 | 0.0156 | 0.0230 | -0.1976 | yes |  |
| b_z0550_t30_a180_x0y1 | predicted_region | 14951 | 0.0170 | 0.0083 | -0.0088 | yes |  |
| b_z0550_t30_a270_x0y1 | predicted_region | 13100 | 0.0165 | 0.0258 | 0.0628 | yes |  |
| b_z0550_t00_a000_x0y2 | predicted_region | 35032 | 0.0175 | 0.0162 | -0.0676 | yes |  |
| b_z0550_t15_a000_x0y2 | predicted_region | 33019 | 0.0168 | 0.0218 | 0.1002 | no | held out |
| b_z0550_t15_a090_x0y2 | predicted_region | 33475 | 0.0166 | 0.0166 | 0.1288 | yes |  |
| b_z0550_t15_a180_x0y2 | predicted_region | 30383 | 0.0172 | 0.0117 | 0.0555 | yes |  |
| b_z0550_t15_a270_x0y2 | predicted_region | 30469 | 0.0163 | 0.0188 | -0.0102 | yes |  |
| b_z0550_t30_a000_x0y2 | predicted_region | 13473 | 0.0170 | 0.0289 | -0.0868 | yes |  |
| b_z0550_t30_a270_x0y2 | predicted_region | 13361 | 0.0162 | 0.0225 | 0.0502 | yes |  |
| b_z0550_t00_a000_x1y2 | predicted_region | 34062 | 0.0160 | 0.0188 | -0.0517 | no | held out |
| b_z0550_t15_a000_x1y2 | predicted_region | 31079 | 0.0166 | 0.0233 | 0.1007 | yes |  |
| b_z0550_t15_a090_x1y2 | predicted_region | 28356 | 0.0164 | 0.0164 | 0.1220 | no | held out |
| b_z0550_t15_a180_x1y2 | predicted_region | 31185 | 0.0166 | 0.0136 | -0.0107 | yes |  |
| b_z0550_t15_a270_x1y2 | predicted_region | 31405 | 0.0159 | 0.0217 | 0.0086 | yes |  |
| b_z0550_t30_a000_x1y2 | predicted_region | 12558 | 0.0165 | 0.0253 | 0.1264 | yes |  |
| b_z0550_t30_a090_x1y2 | predicted_region | 14154 | 0.0163 | 0.0153 | -0.1535 | yes |  |
| b_z0550_t30_a180_x1y2 | predicted_region | 13556 | 0.0164 | 0.0117 | 0.0338 | yes |  |
| b_z0550_t30_a270_x1y2 | predicted_region | 13280 | 0.0154 | 0.0288 | 0.0782 | yes |  |
| b_z0550_t00_a000_x2y2 | predicted_region | 35042 | 0.0167 | 0.0210 | 0.0536 | yes |  |
| b_z0550_t15_a000_x2y2 | predicted_region | 33402 | 0.0163 | 0.0263 | -0.0806 | no | held out |
| b_z0550_t15_a090_x2y2 | predicted_region | 30288 | 0.0174 | 0.0170 | -0.0943 | yes |  |
| b_z0550_t15_a180_x2y2 | predicted_region | 32845 | 0.0167 | 0.0193 | 0.0215 | yes |  |
| b_z0550_t15_a270_x2y2 | predicted_region | 33070 | 0.0167 | 0.0268 | 0.0286 | yes |  |
| b_z0550_t30_a000_x2y2 | predicted_region | 12709 | 0.0170 | 0.0256 | 0.1358 | yes |  |
| b_z0550_t30_a090_x2y2 | predicted_region | 13517 | 0.0165 | 0.0181 | 0.1668 | yes |  |
| b_z0750_t00_a000_x0y0 | predicted_region | 18206 | 0.0314 | 0.0254 | -0.0858 | yes |  |
| b_z0750_t15_a000_x0y0 | predicted_region | 14733 | 0.0307 | 0.0302 | -0.1206 | yes |  |
| b_z0750_t15_a090_x0y0 | predicted_region | 15173 | 0.0301 | 0.0267 | -0.1899 | yes |  |
| b_z0750_t15_a180_x0y0 | predicted_region | 14458 | 0.0312 | 0.0202 | -0.0398 | yes |  |
| b_z0750_t15_a270_x0y0 | predicted_region | 16033 | 0.0314 | 0.0237 | -0.0051 | yes |  |
| b_z0750_t30_a000_x0y0 | predicted_region | 4655 | 0.0323 | 0.0239 | 0.1681 | yes |  |
| b_z0750_t30_a090_x0y0 | predicted_region | 6357 | 0.0283 | 0.0290 | -0.2846 | yes |  |
| b_z0750_t30_a180_x0y0 | predicted_region | 7397 | 0.0334 | 0.0121 | -0.0028 | yes |  |
| b_z0750_t30_a270_x0y0 | predicted_region | 8169 | 0.0319 | 0.0229 | -0.0944 | yes |  |
| b_z0750_t00_a000_x1y0 | predicted_region | 17737 | 0.0300 | 0.0207 | 0.0733 | yes |  |
| b_z0750_t15_a000_x1y0 | predicted_region | 12600 | 0.0297 | 0.0289 | -0.0969 | no | held out |
| b_z0750_t15_a090_x1y0 | predicted_region | 15410 | 0.0294 | 0.0246 | -0.1655 | yes |  |
| b_z0750_t15_a180_x1y0 | predicted_region | 16057 | 0.0307 | 0.0233 | 0.0328 | yes |  |
| b_z0750_t15_a270_x1y0 | predicted_region | 15780 | 0.0295 | 0.0227 | -0.0241 | yes |  |
| b_z0750_t30_a000_x1y0 | predicted_region | 3852 | 0.0287 | 0.0305 | -0.1181 | yes |  |
| b_z0750_t30_a090_x1y0 | predicted_region | 7027 | 0.0278 | 0.0242 | -0.2448 | yes |  |
| b_z0750_t30_a180_x1y0 | predicted_region | 4287 | 0.0314 | 0.0086 | 0.0303 | yes |  |
| b_z0750_t30_a270_x1y0 | predicted_region | 7071 | 0.0290 | 0.0298 | -0.1499 | yes |  |
| b_z0750_t00_a000_x2y0 | predicted_region | 18208 | 0.0300 | 0.0249 | -0.0520 | yes |  |
| b_z0750_t15_a000_x2y0 | predicted_region | 16116 | 0.0310 | 0.0267 | 0.0978 | no | held out |
| b_z0750_t15_a090_x2y0 | predicted_region | 15116 | 0.0310 | 0.0239 | 0.1565 | yes |  |
| b_z0750_t15_a180_x2y0 | predicted_region | 15535 | 0.0302 | 0.0234 | -0.0023 | yes |  |
| b_z0750_t15_a270_x2y0 | predicted_region | 15195 | 0.0313 | 0.0272 | -0.0414 | yes |  |
| b_z0750_t30_a000_x2y0 | predicted_region | 5562 | 0.0336 | 0.0278 | 0.1299 | no | held out |
| b_z0750_t30_a090_x2y0 | predicted_region | 7212 | 0.0298 | 0.0273 | -0.2557 | no | held out |
| b_z0750_t30_a180_x2y0 | predicted_region | 4302 | 0.0327 | 0.0235 | 0.1032 | yes |  |
| b_z0750_t30_a270_x2y0 | predicted_region | 7449 | 0.0304 | 0.0312 | -0.1534 | yes |  |
| b_z0750_t00_a000_x2y1 | predicted_region | 18011 | 0.0298 | 0.0266 | -0.0447 | yes |  |
| b_z0750_t15_a000_x2y1 | predicted_region | 16212 | 0.0321 | 0.0271 | -0.1032 | no | held out |
| b_z0750_t15_a090_x2y1 | predicted_region | 15535 | 0.0304 | 0.0226 | -0.1358 | yes |  |
| b_z0750_t15_a180_x2y1 | predicted_region | 15101 | 0.0313 | 0.0216 | 0.0261 | yes |  |
| b_z0750_t15_a270_x2y1 | predicted_region | 16277 | 0.0305 | 0.0280 | 0.0530 | yes |  |
| b_z0750_t30_a000_x2y1 | predicted_region | 5779 | 0.0307 | 0.0248 | 0.1699 | yes |  |
| b_z0750_t30_a090_x2y1 | predicted_region | 4900 | 0.0314 | 0.0229 | 0.2382 | yes |  |
| b_z0750_t30_a180_x2y1 | predicted_region | 3832 | 0.0316 | 0.0096 | -0.0288 | yes |  |
| b_z0750_t30_a270_x2y1 | predicted_region | 4184 | 0.0301 | 0.0382 | 0.1606 | yes |  |
| b_z0750_t00_a000_x1y1 | predicted_region | 17427 | 0.0301 | 0.0222 | -0.0595 | no | held out |
| b_z0750_t15_a000_x1y1 | predicted_region | 14535 | 0.0304 | 0.0277 | 0.1162 | yes |  |
| b_z0750_t15_a090_x1y1 | predicted_region | 15487 | 0.0316 | 0.0197 | -0.1433 | no | held out |
| b_z0750_t15_a180_x1y1 | predicted_region | 15232 | 0.0296 | 0.0187 | -0.0024 | yes |  |
| b_z0750_t15_a270_x1y1 | predicted_region | 14744 | 0.0293 | 0.0228 | -0.0045 | yes |  |
| b_z0750_t30_a000_x1y1 | predicted_region | 7515 | 0.0328 | 0.0346 | -0.1189 | yes |  |
| b_z0750_t30_a090_x1y1 | predicted_region | 3737 | 0.0276 | 0.0167 | 0.1336 | no | held out |
| b_z0750_t30_a180_x1y1 | predicted_region | 1713 | 0.0295 | 0.0079 | -0.0215 | yes |  |
| b_z0750_t30_a270_x1y1 | predicted_region | 8490 | 0.0284 | 0.0261 | -0.0793 | yes |  |
| b_z0750_t00_a000_x0y1 | predicted_region | 18015 | 0.0309 | 0.0228 | 0.0818 | yes |  |
| b_z0750_t15_a000_x0y1 | predicted_region | 15889 | 0.0293 | 0.0258 | -0.1053 | no | held out |
| b_z0750_t15_a090_x0y1 | predicted_region | 16279 | 0.0300 | 0.0196 | 0.1575 | yes |  |
| b_z0750_t15_a180_x0y1 | predicted_region | 16443 | 0.0307 | 0.0146 | -0.0383 | no | held out |
| b_z0750_t15_a270_x0y1 | predicted_region | 15221 | 0.0310 | 0.0231 | 0.0033 | no | held out |
| b_z0750_t30_a000_x0y1 | predicted_region | 7027 | 0.0315 | 0.0288 | -0.1064 | no | held out |
| b_z0750_t30_a090_x0y1 | predicted_region | 2679 | 0.0293 | 0.0211 | -0.2298 | yes |  |
| b_z0750_t30_a180_x0y1 | predicted_region | 3182 | 0.0333 | 0.0022 | -0.0352 | yes |  |
| b_z0750_t30_a270_x0y1 | predicted_region | 5752 | 0.0297 | 0.0246 | -0.1011 | yes |  |
| b_z0750_t00_a000_x0y2 | predicted_region | 18209 | 0.0308 | 0.0174 | -0.0714 | yes |  |
| b_z0750_t15_a000_x0y2 | predicted_region | 17205 | 0.0301 | 0.0235 | 0.0927 | yes |  |
| b_z0750_t15_a090_x0y2 | predicted_region | 15830 | 0.0322 | 0.0174 | -0.1456 | no | held out |
| b_z0750_t15_a180_x0y2 | predicted_region | 15765 | 0.0305 | 0.0081 | -0.0355 | yes |  |
| b_z0750_t15_a270_x0y2 | predicted_region | 15791 | 0.0308 | 0.0174 | 0.0132 | yes |  |
| b_z0750_t30_a000_x0y2 | predicted_region | 4735 | 0.0316 | 0.0248 | 0.1134 | yes |  |
| b_z0750_t30_a090_x0y2 | predicted_region | 7433 | 0.0321 | 0.0244 | -0.2453 | yes |  |
| b_z0750_t30_a180_x0y2 | predicted_region | 4710 | 0.0300 | 0.0137 | 0.0576 | yes |  |
| b_z0750_t30_a270_x0y2 | predicted_region | 6137 | 0.0318 | 0.0241 | 0.0869 | yes |  |
| b_z0750_t00_a000_x1y2 | predicted_region | 17735 | 0.0312 | 0.0163 | -0.0545 | yes |  |
| b_z0750_t15_a000_x1y2 | predicted_region | 15416 | 0.0300 | 0.0258 | -0.1090 | yes |  |
| b_z0750_t15_a090_x1y2 | predicted_region | 16425 | 0.0298 | 0.0133 | -0.1184 | yes |  |
| b_z0750_t15_a180_x1y2 | predicted_region | 15599 | 0.0309 | 0.0140 | 0.0017 | yes |  |
| b_z0750_t15_a270_x1y2 | predicted_region | 15273 | 0.0295 | 0.0240 | 0.0190 | yes |  |
| b_z0750_t30_a000_x1y2 | predicted_region | 5179 | 0.0311 | 0.0245 | 0.1477 | no | held out |
| b_z0750_t30_a090_x1y2 | predicted_region | 6772 | 0.0296 | 0.0190 | 0.2067 | yes |  |
| b_z0750_t30_a180_x1y2 | predicted_region | 7127 | 0.0311 | 0.0098 | 0.0368 | yes |  |
| b_z0750_t30_a270_x1y2 | predicted_region | 4255 | 0.0289 | 0.0283 | -0.0832 | yes |  |
| b_z0750_t00_a000_x2y2 | predicted_region | 18208 | 0.0314 | 0.0197 | -0.0344 | yes |  |
| b_z0750_t15_a000_x2y2 | predicted_region | 16074 | 0.0315 | 0.0305 | -0.1035 | yes |  |
| b_z0750_t15_a090_x2y2 | predicted_region | 16872 | 0.0306 | 0.0185 | -0.1126 | yes |  |
| b_z0750_t15_a180_x2y2 | predicted_region | 15470 | 0.0308 | 0.0196 | -0.0234 | no | held out |
| b_z0750_t15_a270_x2y2 | predicted_region | 15788 | 0.0308 | 0.0234 | -0.0270 | yes |  |
| b_z0750_t30_a000_x2y2 | predicted_region | 766 | 0.0319 | 0.0308 | 0.1323 | no | too small |
| b_z0750_t30_a090_x2y2 | predicted_region | 6984 | 0.0314 | 0.0151 | -0.1720 | yes |  |
| b_z0750_t30_a180_x2y2 | predicted_region | 8159 | 0.0307 | 0.0154 | -0.0867 | yes |  |
| b_z0750_t30_a270_x2y2 | predicted_region | 5589 | 0.0310 | 0.0283 | -0.1149 | yes |  |
| b_z0950_t00_a000_x0y0 | predicted_region | 10862 | 0.0499 | 0.0261 | 0.0822 | yes |  |
| b_z0950_t15_a000_x0y0 | predicted_region | 5794 | 0.0487 | 0.0292 | -0.0864 | yes |  |
| b_z0950_t15_a090_x0y0 | predicted_region | 10171 | 0.0487 | 0.0297 | 0.2145 | no | held out |
| b_z0950_t15_a180_x0y0 | predicted_region | 8366 | 0.0473 | 0.0282 | 0.0024 | yes |  |
| b_z0950_t15_a270_x0y0 | predicted_region | 8890 | 0.0476 | 0.0286 | -0.0267 | yes |  |
| b_z0950_t30_a000_x0y0 | predicted_region | 4366 | 0.0524 | 0.0358 | 0.2278 | no | held out |
| b_z0950_t30_a090_x0y0 | predicted_region | 5188 | 0.0513 | 0.0232 | 0.2955 | no | held out |
| b_z0950_t30_a180_x0y0 | predicted_region | 518 | 0.0454 | 0.0699 | -0.4788 | no | too small |
| b_z0950_t30_a270_x0y0 | predicted_region | 1109 | 0.0471 | 0.0646 | -0.2951 | yes |  |
| b_z0950_t00_a000_x1y0 | predicted_region | 10668 | 0.0462 | 0.0263 | 0.0739 | yes |  |
| b_z0950_t15_a000_x1y0 | predicted_region | 6715 | 0.0482 | 0.0276 | 0.1055 | yes |  |
| b_z0950_t15_a090_x1y0 | predicted_region | 8731 | 0.0489 | 0.0290 | 0.2147 | no | held out |
| b_z0950_t15_a180_x1y0 | predicted_region | 8918 | 0.0470 | 0.0162 | -0.0599 | yes |  |
| b_z0950_t15_a270_x1y0 | predicted_region | 9399 | 0.0459 | 0.0279 | 0.0477 | yes |  |
| b_z0950_t30_a000_x1y0 | predicted_region | 1947 | 0.0513 | 0.0150 | -0.0096 | yes |  |
| b_z0950_t30_a090_x1y0 | predicted_region | 4622 | 0.0468 | 0.0253 | 0.3008 | yes |  |
| b_z0950_t30_a180_x1y0 | predicted_region | 1985 | 0.0517 | 0.0118 | 0.0103 | yes |  |
| b_z0950_t30_a270_x1y0 | predicted_region | 918 | 0.0516 | 0.0472 | 0.2951 | no | too small |
| b_z0950_t00_a000_x2y0 | predicted_region | 10831 | 0.0505 | 0.0271 | -0.0538 | yes |  |
| b_z0950_t15_a000_x2y0 | predicted_region | 10492 | 0.0506 | 0.0271 | 0.1258 | yes |  |
| b_z0950_t15_a090_x2y0 | predicted_region | 9785 | 0.0482 | 0.0325 | 0.1948 | yes |  |
| b_z0950_t15_a180_x2y0 | predicted_region | 9100 | 0.0495 | 0.0209 | -0.0132 | yes |  |
| b_z0950_t15_a270_x2y0 | predicted_region | 8445 | 0.0477 | 0.0288 | -0.0781 | yes |  |
| b_z0950_t30_a000_x2y0 | predicted_region | 725 | 0.0408 | 0.1670 | -1.2980 | no | too small |
| b_z0950_t30_a090_x2y0 | predicted_region | 1133 | 0.0509 | 0.0330 | -0.2526 | yes |  |
| b_z0950_t30_a180_x2y0 | predicted_region | 1349 | 0.0497 | 0.0094 | -0.0012 | yes |  |
| b_z0950_t30_a270_x2y0 | predicted_region | 1604 | 0.0469 | 0.0351 | -0.2438 | yes |  |
| b_z0950_t00_a000_x2y1 | predicted_region | 10839 | 0.0478 | 0.0224 | -0.0430 | yes |  |
| b_z0950_t15_a000_x2y1 | predicted_region | 8902 | 0.0478 | 0.0295 | -0.1236 | no | held out |
| b_z0950_t15_a090_x2y1 | predicted_region | 9111 | 0.0479 | 0.0207 | -0.1493 | no | held out |
| b_z0950_t15_a180_x2y1 | predicted_region | 8591 | 0.0467 | 0.0201 | 0.0263 | yes |  |
| b_z0950_t15_a270_x2y1 | predicted_region | 8593 | 0.0480 | 0.0234 | 0.0630 | no | held out |
| b_z0950_t30_a000_x2y1 | predicted_region | 1747 | 0.0451 | 0.0084 | 0.0751 | yes |  |
| b_z0950_t30_a090_x2y1 | predicted_region | 4612 | 0.0515 | 0.0192 | 0.2493 | yes |  |
| b_z0950_t30_a180_x2y1 | predicted_region | 491 | 0.0433 | 0.1006 | -0.2996 | no | too small |
| b_z0950_t30_a270_x2y1 | predicted_region | 2967 | 0.0498 | 0.0315 | 0.1747 | yes |  |
| b_z0950_t00_a000_x1y1 | predicted_region | 10701 | 0.0484 | 0.0252 | 0.0702 | yes |  |
| b_z0950_t15_a000_x1y1 | predicted_region | 8609 | 0.0488 | 0.0253 | 0.1247 | no | held out |
| b_z0950_t15_a090_x1y1 | predicted_region | 8735 | 0.0463 | 0.0223 | -0.1703 | yes |  |
| b_z0950_t15_a180_x1y1 | predicted_region | 8656 | 0.0476 | 0.0166 | 0.0186 | yes |  |
| b_z0950_t15_a270_x1y1 | predicted_region | 7097 | 0.0462 | 0.0245 | 0.0223 | no | held out |
| b_z0950_t30_a000_x1y1 | predicted_region | 2999 | 0.0501 | 0.0430 | 0.2710 | yes |  |
| b_z0950_t30_a090_x1y1 | predicted_region | 2548 | 0.0456 | 0.0245 | -0.2848 | yes |  |
| b_z0950_t30_a180_x1y1 | predicted_region | 1751 | 0.0428 | 0.0061 | -0.0150 | no | held out |
| b_z0950_t30_a270_x1y1 | predicted_region | 3270 | 0.0453 | 0.0252 | -0.1596 | yes |  |
| b_z0950_t00_a000_x0y1 | predicted_region | 10840 | 0.0469 | 0.0220 | 0.0827 | yes |  |
| b_z0950_t15_a000_x0y1 | predicted_region | 7942 | 0.0485 | 0.0271 | 0.1473 | yes |  |
| b_z0950_t15_a090_x0y1 | predicted_region | 9376 | 0.0482 | 0.0204 | 0.1657 | yes |  |
| b_z0950_t15_a180_x0y1 | predicted_region | 9459 | 0.0504 | 0.0159 | -0.0858 | yes |  |
| b_z0950_t15_a270_x0y1 | predicted_region | 9861 | 0.0469 | 0.0278 | 0.0340 | yes |  |
| b_z0950_t30_a000_x0y1 | predicted_region | 1484 | 0.0474 | 0.0247 | -0.1464 | no | held out |
| b_z0950_t30_a090_x0y1 | predicted_region | 2898 | 0.0485 | 0.0392 | 0.4180 | yes |  |
| b_z0950_t30_a180_x0y1 | predicted_region | 554 | 0.0445 | 0.0152 | -0.0540 | no | too small |
| b_z0950_t30_a270_x0y1 | predicted_region | 935 | 0.0501 | 0.0520 | -0.4057 | no | too small |
| b_z0950_t00_a000_x0y2 | predicted_region | 10972 | 0.0483 | 0.0175 | 0.0719 | yes |  |
| b_z0950_t15_a000_x0y2 | predicted_region | 8171 | 0.0481 | 0.0234 | 0.1306 | yes |  |
| b_z0950_t15_a090_x0y2 | predicted_region | 9083 | 0.0515 | 0.0221 | -0.1762 | yes |  |
| b_z0950_t15_a180_x0y2 | predicted_region | 6571 | 0.0498 | 0.0155 | -0.1129 | yes |  |
| b_z0950_t15_a270_x0y2 | predicted_region | 8932 | 0.0507 | 0.0134 | 0.0162 | yes |  |
| b_z0950_t30_a000_x0y2 | predicted_region | 660 | 0.0505 | 0.0525 | -0.1184 | no | too small |
| b_z0950_t30_a090_x0y2 | predicted_region | 5086 | 0.0534 | 0.0123 | -0.1740 | yes |  |
| b_z0950_t30_a180_x0y2 | predicted_region | 4062 | 0.0509 | 0.0054 | -0.0139 | no | held out |
| b_z0950_t30_a270_x0y2 | predicted_region | 3837 | 0.0504 | 0.0243 | 0.1283 | yes |  |
| b_z0950_t00_a000_x1y2 | predicted_region | 10586 | 0.0465 | 0.0272 | -0.0591 | yes |  |
| b_z0950_t15_a000_x1y2 | predicted_region | 10128 | 0.0492 | 0.0229 | 0.1213 | yes |  |
| b_z0950_t15_a090_x1y2 | predicted_region | 8893 | 0.0486 | 0.0214 | 0.1517 | yes |  |
| b_z0950_t15_a180_x1y2 | predicted_region | 9317 | 0.0488 | 0.0102 | -0.0202 | yes |  |
| b_z0950_t15_a270_x1y2 | predicted_region | 8855 | 0.0497 | 0.0250 | 0.0429 | yes |  |
| b_z0950_t30_a000_x1y2 | predicted_region | 2112 | 0.0490 | 0.0500 | -0.3427 | yes |  |
| b_z0950_t30_a090_x1y2 | predicted_region | 3213 | 0.0435 | 0.0191 | 0.2428 | no | held out |
| b_z0950_t30_a180_x1y2 | predicted_region | 1621 | 0.0456 | 0.0379 | 0.3125 | yes |  |
| b_z0950_t30_a270_x1y2 | predicted_region | 3408 | 0.0468 | 0.0210 | -0.0597 | yes |  |
| b_z0950_t00_a000_x2y2 | predicted_region | 10969 | 0.0500 | 0.0218 | -0.0359 | yes |  |
| b_z0950_t15_a000_x2y2 | predicted_region | 9261 | 0.0510 | 0.0216 | 0.1124 | yes |  |
| b_z0950_t15_a090_x2y2 | predicted_region | 8823 | 0.0509 | 0.0187 | 0.1297 | yes |  |
| b_z0950_t15_a180_x2y2 | predicted_region | 9039 | 0.0476 | 0.0225 | -0.0557 | yes |  |
| b_z0950_t15_a270_x2y2 | predicted_region | 8851 | 0.0493 | 0.0262 | 0.0854 | yes |  |
| b_z0950_t30_a000_x2y2 | predicted_region | 706 | 0.0480 | 0.0461 | -0.2893 | no | too small |
| b_z0950_t30_a090_x2y2 | predicted_region | 5002 | 0.0486 | 0.0147 | -0.2026 | no | held out |
| b_z0950_t30_a180_x2y2 | predicted_region | 886 | 0.0450 | 0.0656 | -0.0893 | no | too small |
| b_z0950_t30_a270_x2y2 | predicted_region | 510 | 0.0452 | 0.0640 | 0.0639 | no | too small |

### Errors against the known truth

| pass | rotation error (deg) | translation error (mm) | scale | scale error |
|---|---|---|---|---|
| pass1 | 0.0045 | 0.0187 | 1.000011 | 0.000011 |
| pass2 | 0.0046 | 0.0192 | 1.000011 | 0.000011 |

## Comparison of two sessions

### Model: rigid

| quantity | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0242 | 0.0132 |
| RMS offset residual (mm) | 0.1249 | 0.0565 |
| poses used | 179 | 178 |

| held-out poses | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0237 | 0.0081 |
| RMS offset residual (mm) | 0.1321 | 0.0664 |
| held-out poses | 45 | 44 |

Session B minimizing backlash has the smaller residual RMS of the two (173 poses in common).
Relative transform between the two solutions: rotation 0.0258 deg, translation 0.2740 mm, scale ratio 1.000000.

### Model: similarity

| quantity | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0242 | 0.0132 |
| RMS offset residual (mm) | 0.1249 | 0.0565 |
| poses used | 179 | 178 |

| held-out poses | A ignoring backlash | B minimizing backlash |
|---|---|---|
| RMS normal residual (deg) | 0.0237 | 0.0081 |
| RMS offset residual (mm) | 0.1317 | 0.0665 |
| held-out poses | 45 | 44 |

Session B minimizing backlash has the smaller residual RMS of the two (173 poses in common).
Relative transform between the two solutions: rotation 0.0258 deg, translation 0.2800 mm, scale ratio 0.999973.

## Figures

- [normal_residual_field.png](analysis_A/figures/rigid/normal_residual_field.png)
- [offset_residual_field.png](analysis_A/figures/rigid/offset_residual_field.png)
- [pixel_residuals_b_z0550_t15_a180_x2y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0550_t15_a180_x2y0.png)
- [pixel_residuals_b_z0750_t00_a000_x1y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0750_t00_a000_x1y1.png)
- [pixel_residuals_b_z0950_t30_a000_x1y2.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a000_x1y2.png)
- [pixel_residuals_b_z0950_t30_a000_x2y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a000_x2y0.png)
- [pixel_residuals_b_z0950_t30_a090_x0y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a090_x0y1.png)
- [pixel_residuals_b_z0950_t30_a180_x0y0.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a180_x0y0.png)
- [pixel_residuals_b_z0950_t30_a180_x1y2.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a180_x1y2.png)
- [pixel_residuals_b_z0950_t30_a270_x0y1.png](analysis_A/figures/rigid/pixel_residuals_b_z0950_t30_a270_x0y1.png)
- [pose_residuals.png](analysis_A/figures/rigid/pose_residuals.png)
- [normal_residual_field.png](analysis_A/figures/similarity/normal_residual_field.png)
- [offset_residual_field.png](analysis_A/figures/similarity/offset_residual_field.png)
- [pixel_residuals_b_z0750_t15_a180_x2y2.png](analysis_A/figures/similarity/pixel_residuals_b_z0750_t15_a180_x2y2.png)
- [pixel_residuals_b_z0950_t15_a180_x1y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t15_a180_x1y0.png)
- [pixel_residuals_b_z0950_t30_a000_x1y2.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a000_x1y2.png)
- [pixel_residuals_b_z0950_t30_a000_x2y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a000_x2y0.png)
- [pixel_residuals_b_z0950_t30_a090_x0y1.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a090_x0y1.png)
- [pixel_residuals_b_z0950_t30_a180_x0y0.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a180_x0y0.png)
- [pixel_residuals_b_z0950_t30_a180_x1y2.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a180_x1y2.png)
- [pixel_residuals_b_z0950_t30_a270_x0y1.png](analysis_A/figures/similarity/pixel_residuals_b_z0950_t30_a270_x0y1.png)
- [pose_residuals.png](analysis_A/figures/similarity/pose_residuals.png)
- [comparison.png](comparison/comparison.png)
