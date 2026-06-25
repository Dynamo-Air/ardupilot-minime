# AS-10 MiniMe Mast Material Upgrade Procedure

This document describes the procedure for upgrading the mast material from aluminum to steel 4340 or titanium, including coordination requirements, verification steps, and parameter updates.

---

## 1. Mast Material Specifications

### 1.1 Material Capabilities

| Material | Pitch Rate Limit | Safety Factor | Angular Rate (rad/s) | Notes |
|----------|------------------|---------------|---------------------|-------|
| Aluminum (Current) | 30 deg/s | SF > 1.0 | 0.5 | Baseline configuration |
| Steel 4340 | 57.3 deg/s | SF > 3.0 | 1.0 | Recommended upgrade |
| Titanium | 57.3 deg/s | SF > 3.0 | 1.0 | Similar to steel with weight reduction |

Reference: SPECS.md Section 6.2 and Section 26.5

### 1.2 Current Configuration

The AS-10 MiniMe baseline configuration assumes an aluminum mast. This limits pitch rate to 30 deg/s to maintain a safety factor greater than 1.0 under gyroscopic loading. The pitch angular acceleration available is 75.76 deg/s^2 at T/W 1.2.

### 1.3 Upgrade Benefits

Upgrading to steel 4340 or titanium allows:
- Pitch rate increase from 30 deg/s to 57.3 deg/s
- Safety factor improvement from SF > 1.0 to SF > 3.0
- Improved agility without structural risk

---

## 2. Pre-Upgrade Requirements

### 2.1 Structural Analysis Coordination

Before any mast material change, coordinate with the structural analysis team. The following deliverables are required:

| Deliverable | Description | Owner |
|-------------|-------------|-------|
| Stress Analysis Report | Static and dynamic stress analysis for new mast material under maximum loading conditions | Structural Team |
| Safety Factor Calculations | SF calculations for pitch rates from 30 to 60 deg/s at 5 deg/s increments | Structural Team |
| Fatigue Analysis | Fatigue life assessment for new material under expected flight profiles | Structural Team |
| Approval Signature | Written approval from structural lead for new pitch rate limit | Structural Lead |

### 2.2 Required Approvals

| Approval | Authority | Requirement |
|----------|-----------|-------------|
| Structural Analysis Complete | Structural Lead | All deliverables in Section 2.1 complete |
| Safety Factor Verified | Chief Engineer | SF > 3.0 confirmed for new pitch rate limit |
| Configuration Change Approved | Program Manager | Formal approval for configuration change |

### 2.3 Documentation Requirements

Before proceeding with the upgrade, ensure:
1. Structural analysis report filed in project documentation system
2. Safety factor calculations reviewed and approved
3. Configuration change request approved and documented
4. Updated flight envelope documented

---

## 3. Upgrade Procedure

### 3.1 Physical Mast Replacement

The physical mast replacement is performed by the mechanical team following the applicable mechanical assembly procedures. This document covers only the software and parameter changes.

Mechanical procedure reference: Contact mechanical team lead for current assembly procedure document.

### 3.2 Pre-Change Verification

Before modifying parameters:

| Step | Action | Verification |
|------|--------|--------------|
| 1 | Confirm new mast installed | Visual inspection, mechanical team sign-off |
| 2 | Confirm structural analysis complete | Stress analysis report available |
| 3 | Confirm safety factor approved | SF > 3.0 for target pitch rate confirmed |
| 4 | Record current parameter values | Backup minime_flight.param before changes |

### 3.3 Parameter Update Procedure

Follow these steps to update pitch rate parameters:

1. Connect GCS (Mission Planner or MAVProxy)
2. Navigate to Full Parameter List
3. Locate ATC_RAT_PIT_MAX parameter
4. Record current value (should be 30)
5. Change ATC_RAT_PIT_MAX to new approved value (up to 57.3 for steel/titanium)
6. Write parameter to flight controller
7. Reboot flight controller
8. Verify parameter persisted after reboot
9. Update minime_flight.param file (see Section 4)

---

## 4. Parameter Updates

### 4.1 ATC_RAT_PIT_MAX Change

| Parameter | Aluminum Value | Steel/Titanium Value | Unit |
|-----------|----------------|---------------------|------|
| ATC_RAT_PIT_MAX | 30 | 57.3 (or per structural approval) | deg/s |

The new value must not exceed the structurally approved pitch rate limit from the stress analysis report. If structural analysis approves a value less than 57.3 deg/s, use the approved value.

### 4.2 minime_flight.param Update

After updating parameters on the flight controller, update the parameter file for consistency.

Location: `params/minime_flight.param`

Change the following section:

From:
```
# Pitch Rate Limit (SPECS.md Section 6.2)
# Aluminum mast constraint; steel mast supports 57.3 deg/s
ATC_RAT_PIT_MAX,30
```

To (example for steel mast):
```
# Pitch Rate Limit (SPECS.md Section 6.2)
# Steel 4340 mast installed; SF > 3.0 at 57.3 deg/s
# Structural analysis approval: [DATE] [APPROVAL REFERENCE]
ATC_RAT_PIT_MAX,57.3
```

### 4.3 Configuration Traceability

Update the flight parameter file header to reflect the configuration change:

| Field | Update |
|-------|--------|
| Mast Material | Change from "Aluminum (assumed)" to "Steel 4340" or "Titanium" |
| Structural Approval | Add approval date and reference number |
| Effective Date | Date of parameter change |

---

## 5. Post-Upgrade Verification

### 5.1 Ground Test Requirements

Before flight testing with the new pitch rate limit:

| Test | Acceptance Criteria | Notes |
|------|---------------------|-------|
| Parameter Verification | ATC_RAT_PIT_MAX matches structural approval | GCS readback |
| Servo Range Check | Full pitch command range achieved without saturation | Bench test |
| Control Response | Pitch step response stable | Ground run, rotors turning |

### 5.2 Phase W Validation Requirements

If the mast material change occurs before initial flight testing, include in Phase W whirl stand validation:

| Test | Objective | Acceptance Criteria |
|------|-----------|---------------------|
| Pitch Rate Step | Verify pitch rate response at new limit | No structural anomalies |
| Gyroscopic Loading | Verify mast stress under maximum pitch rate | Strain gauge data within limits (if instrumented) |
| Control Stability | Verify attitude control stable at new rate | Pitch tracking error < 10% |

### 5.3 Flight Test Build-Up

Follow incremental flight test approach for the new pitch rate limit:

| Phase | Pitch Rate | Objective |
|-------|------------|-----------|
| 1 | 30 deg/s | Baseline verification (matches previous limit) |
| 2 | 40 deg/s | Initial increase, pilot evaluation |
| 3 | 50 deg/s | Intermediate evaluation |
| 4 | 57.3 deg/s | Full capability verification |

At each phase:
- Monitor structural response via IMU vibration data
- Evaluate handling qualities
- Verify no anomalous behavior
- Document pilot feedback

Do not proceed to next phase if any anomalies observed. Consult structural team before continuing.

---

## 6. Related Documentation

| Document | Section | Content |
|----------|---------|---------|
| SPECS.md | Section 6.2 | Pitch Rate Limit (Material Dependent) |
| SPECS.md | Section 26.5 | Mast Material Constraints |
| SPECS.md | Section 26.3 | Gyroscopic Moment at 30 deg/s Body Rate |
| STATE.md | Task 6.4 | Mast Material Upgrade Path checklist |
| minime_flight.param | Line 106-108 | ATC_RAT_PIT_MAX parameter |

---

## 7. Revision History

| Date | Revision | Description | Author |
|------|----------|-------------|--------|
| 2026-06-25 | 1.0 | Initial document creation | ArduPilot Development |

---

End of Document
