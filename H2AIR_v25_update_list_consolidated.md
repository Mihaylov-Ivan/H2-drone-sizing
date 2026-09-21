# H2AIR-4+1 application v25: consolidated update list

Status: 20 September 2026. Aircraft model run of 20 September 2026 (configuration hash 8e916a8d). KPI 3 stays at "at most 485 W", with 433 W as the design value.

All draft replacements below were checked against the v25 text: each "Replace" string occurs exactly once, and the parts can be applied in any order. Word counts after all parts: Summary 199, S7 1986, S8 1986, S9 1668, S12 1962.

**Tag key**

- S7 to S12: sections of the application draft. B1, B2, B3: Annexes B1, B2, B3. AnxC, AnxD, AnxE: Annexes C, D, E.
- Part A: structural and writing-style edits. Part M: edits caused by the new model run. Part P: internal to-dos and placeholders. Part DEC: decisions and inputs still needed from you.
- Renamed since the last list: C.1 to C.4 are now AnxC.1 to AnxC.4, D.1 to D.3 are AnxD.1 to AnxD.3, E.1 is AnxE.1, Part C is Part M, Part D is Part P, Part B is Part DEC. W.1, W.2, B.1, B.2, B.3, B.9, B.10, B.11 moved to Part P. B3.5 merged into M4.4.

---



# Part A: can be done now



## A1. Structural changes



### Innovation framing and endurance against batteries

**S7.1** Replace "H2AIR-4+1 will be a system-level integration innovation, which will develop the aircraft around a proven commercial hydrogen power subsystem so that a small VTOL UAV can fly a two-hour mission with a 1.2 kg payload."
With "H2AIR-4+1 will enable a VTOL UAV below 15 kg to fly a two-hour mission on hydrogen with a 1.2 kg payload, by developing the aircraft around a proven commercial fuel-cell power subsystem."

**S7.10** Replace "for 2.7 kg (Annex A3), whereas the same energy in batteries would weigh roughly two to five times as much."
With "for 2.7 kg (Annex A3). The same energy in batteries would weigh roughly two to five times as much. The complete energy system will thus rise from about 165 Wh per kilogram, which is battery level, to about 260 and 325 Wh per kilogram (Annex B2)."

**S7.12** Replace "Compared with the battery VTOL aircraft, it will offer two hours with payload, refuelling on the aircraft in minutes in place of pack charging, and endurance growth through cylinder size alone. Compared with all of them, it will add a printed, modular airframe that can be adapted and produced in series without new tooling, and a documented integration package that will allow other electric platforms to adopt the same powertrain."
With "Compared with the battery VTOL aircraft, it will match their best endurance with its smallest cylinder and will add refuelling in minutes and endurance growth through cylinder size alone. Compared with all of them, it will add the printed, modular airframe and the open integration package."

**S12.1** Replace "A battery aircraft with 90 minutes of endurance will need about seven bases and will wait for charging between flights, whereas the demonstrator configuration of H2AIR-4+1 will need five and the long-endurance variant three to four, and both will be refuelled on the aircraft within minutes between sorties."
With "A battery aircraft with 90 to 120 minutes of endurance will need five to seven bases and will wait for charging between flights. The demonstrator configuration of H2AIR-4+1 will need five and the long-endurance variant three to four, and both will be refuelled on the aircraft within minutes between sorties."

**B2.2** Annex B2, section 3, put at the end: "Specific energy of the energy system (C): the 5.34 kg energy system of section 6.7 supplies the 887 Wh mission, about 165 Wh/kg. With the 6.8 L cylinder (3.9 kg, 2336 Wh, 140 g of hydrogen, Annex A3) it supplies 2003 Wh (2336 x 0.85 + 17) at 7.70 kg, about 260 Wh/kg. With the 9.0 L cylinder (4.3 kg, 3091 Wh, 186 g) it supplies 2644 Wh at 8.14 kg, about 325 Wh/kg. The larger bay structure is excluded."

**B2.3** Annex B2, section 6.1, put at the end: "Long-endurance variant (post-project): the 6.8 L and 9.0 L cylinders have diameters of 160 mm and 173 mm (Annex A3) against 115 mm for the selected cylinder, so the variant will use a wider printed fuselage section. Its mass, stall margin and cruise demand will be resolved with the sizing model in Task 6."

### KPI 1 baseline and reference flight

**S7.15** Replace "The baseline is the host tailsitter's measured battery endurance (Annex C6), and verification will rest on a battery-only reference flight under the same profile followed by"
With "The baseline is the 90 to 120 minutes of the best battery aircraft named above. Verification will rest on a battery-only reference flight of the same airframe and profile with a battery pack of the same mass as the hydrogen energy system, followed by"

**S9.1** Replace "The aircraft will fly a two-hour mission with payload, where the battery-electric host flies [NEEDED FROM HYDROGENERA: measured minutes], and the gain will be verified against a battery-only reference flown under the same profile."
With "The aircraft will fly a two-hour mission with payload and will be refuelled in minutes, and the gain will be verified against a battery-only reference of the same airframe and the same energy-system mass, flown under the same profile."

**S10.2** Replace "The task will next fly the battery-only reference mission under the same profile,"
With "The task will next fly the battery-only reference mission of the same airframe under the same profile, with a battery pack of the same mass as the hydrogen energy system,"

**B2.4** Annex B2, section 7, KPI 1: replace "Test: battery-only reference flight and three hydrogen missions on the same profile"
With "Test: battery-only reference flight of the same airframe with a battery pack of the same mass as the hydrogen energy system (5.34 kg, section 6.7), and three hydrogen missions on the same profile"

### TRL 6 carry-over and re-scaled propulsion

**S7.13** Replace "On this basis, the airframe, the powertrain installation"
With "On this basis, the airframe, the re-scaled propulsion, the powertrain installation"

**S8.1** Replace "has flown outdoors at mission mass and with the mission payload, and it uses the flight controller, propulsion and power-distribution architecture that the new aircraft will carry over."
With "has flown outdoors at its design mass of 5.5 kg with the 1.2 kg mission payload, and the new aircraft will carry over its flight control, avionics and power-measurement architecture."

**S8.2** Replace "the 4+1 airframe, the transition to wing-borne flight and the integration"
With "the 4+1 airframe with its re-scaled propulsion, the transition to wing-borne flight and the integration"

**S8.5** Replace "It will be carried into the new aircraft as the proven baseline."
With "Its flight control, avionics, power measurement and integration method will be carried into the new aircraft as the proven baseline, whereas the motors, propellers and speed controllers will be larger commercial products on a 48 V bus. The re-scaled propulsion will therefore start at TRL 3 with the airframe and will be validated on the thrust stand in Task 2."

**B2.1** Annex B2, section 1, put after "power-distribution and measurement architecture.": "New at the larger scale: lift and cruise motors, propellers and speed controllers on a 48 V bus (section 6.3)."

**AnxE.1** Annex E cover note: replace "the new airframe and hydrogen integration at TRL 3"
With "the new airframe with its re-scaled propulsion and the hydrogen integration at TRL 3"

### Missing scored content

**S9.3** Put this new paragraph after "will be offered for trial on a Pioneer platform.", or in the portal's own field if it has one for the digital-hub bonus:
"Beyond these meetings, Hydrogenera proposes a contribution to the IH-MIE digital hub. The integration package of H2AIR-4+1 will be offered as the first entry of an open integration library on the hub, in which each funded project will publish its non-confidential interface sheet, sizing method and commissioning checklist in one common format. A register of available hydrogen powertrain modules and host platforms will sit beside the library, so that a vehicle developer in one region will be able to find a matching module in another."

### Cross-document consistency and confidentiality

**B3.2** Annex B3, Summary A17: replace "will be requested from the fuel-cell supplier at Task 1 and recorded here"
With "will be requested from the fuel-cell supplier during the project and released with the Task 6 cost model"

**AnxC.2** Annex C1 note: replace "is shown separately in Drawing 03A" with "is shown separately in Annex C2"

**AnxD.3** Annex D Attachment 1: delete the "Timeline" slide and both "Hydrogen supply" slides. Keep the title, setup scheme, proof of concept, performance, results and summary slides.

## A2. Writing-style changes



### Future tense

**S7.3** Replace "airframe that carries this powertrain" with "airframe that will carry this powertrain"

**S7.4** Replace "These four parts respond to a limit" with "These four parts will respond to a limit"

**S7.5** Replace "each of which reduces the fuel-cell size" with "each of which will reduce the fuel-cell size"

**S7.6** Replace "The first choice is wing-borne cruise." with "The first choice will be wing-borne cruise."

**S7.9** Replace "The third choice concerns endurance growth, which will come from the cylinder alone because the first two choices fix the size"
With "The third choice will concern endurance growth, which will come from the cylinder alone because the first two choices will fix the size"

**S7.11** Replace "and this manufacturing route is an innovation in its own right" with "and this manufacturing route will be an innovation in its own right"

**S12.3** Replace "the hydrogen aircraft is positioned at" with "the hydrogen aircraft will be positioned at"

**S12.7** Replace "is left out of these totals until Task 6 has set the tariff. The projection depends on"
With "will enter these totals once Task 6 has set the tariff. The projection will depend on"

**S12.9** Three replacements:

- "The technical target is one" with "The technical target will be one"
- "The performance targets are an endurance" with "The performance targets will be an endurance"
- "The commercial targets are at least" with "The commercial targets will be at least"



### Long sentences split (new model figures already included)

**S7.2** Replace "The first part will be a hydrogen-hybrid powertrain, in which an 800 W air-cooled PEM fuel cell will supply the steady cruise power, a small buffer battery will supply the short peaks of take-off, transition and landing, and an energy management system will divide the load"
With "The first part will be a hydrogen-hybrid powertrain. In it, an 800 W air-cooled PEM fuel cell will supply the steady cruise power and a small buffer battery will supply the short peaks of take-off, transition and landing. An energy management system will divide the load"

**S7.7** Replace "485 W (Annex B2), whereas a hovering multirotor" with "433 W (Annex B2). A hovering multirotor"

**S7.8** Replace "and the second choice addresses this. Where a fuel cell alone would have to be a 5 kW unit serving a 485 W cruise, H2AIR-4+1 will pair"
With "and the second choice will address this. A fuel cell alone would have to be a 5 kW unit serving a 433 W cruise. H2AIR-4+1 will instead pair"

**S7.14** Replace "Two activities will run alongside this path: the structural verification of the printed and carbon-fibre parts (Annex B2), and hydrogen safety, which will carry"
With "Two activities will run alongside this path. The first will be the structural verification of the printed and carbon-fibre parts (Annex B2). The second will be hydrogen safety, which will carry"

**S8.9** Replace "Its gate will be passed when four results have been measured: a mission-usable energy of at least 875 Wh, coverage of the 5 kW peak profile by the battery within its voltage and temperature limits, a thrust of at least 62 N from each lift motor, and a working safety cut-off on the bench."
With "Its gate will be passed when four results have been measured. The first two will be a mission-usable energy of at least 875 Wh and coverage of the 5 kW peak profile by the battery within its voltage and temperature limits. The other two will be a thrust of at least 60 N from each lift motor and a working safety cut-off on the bench."

**S8.10** Replace "The gate of this step will be passed when endurance has been measured against the battery-only reference (KPI 1), the cruise electrical demand is at or below 485 W (KPI 3), a commanded fuel-cell shutdown has been followed by a battery landing with reserve (KPI 5) and the three missions have been flown without a critical fault (KPI 6)."
With "The gate of this step will be passed on four results. Endurance will have been measured against the battery-only reference (KPI 1), and the cruise electrical demand will be at or below 485 W (KPI 3). A commanded fuel-cell shutdown will have been followed by a battery landing with reserve (KPI 5), and the three missions will have been flown without a critical fault (KPI 6)."

**S9.2** Replace "Industrial installations completed in 2026, namely a 60 kW pure-hydrogen supply for generator cooling, a 90 kW system for an automotive paint line in Poland and a 500 kW system for the cement industry, have given the team"
With "Three industrial installations were completed in 2026: a 60 kW pure-hydrogen supply for generator cooling, a 90 kW system for an automotive paint line in Poland and a 500 kW system for the cement industry. They have given the team"

**S12.2** Replace "because the printed route is also the series route (Section 7), so Hydrogenera plans with"
With "because the printed route will also be the series route (Section 7). Hydrogenera therefore plans with"

**S12.4** Replace "and with the customer interviews, and it will derive the cost per inspected kilometre"
With "and with the customer interviews. It will also derive the cost per inspected kilometre"

**S12.5** Replace "(Annex F: F12), and the gas transmission network has" with "(Annex F: F12). The gas transmission network has"

**S12.6** Replace "give the aircraft dual-use potential in civil protection, wildfire watch, search and rescue, and border surveillance, which Hydrogenera will approach after the civil launch"
With "will give the aircraft dual-use potential in civil protection, wildfire watch, search and rescue, and border surveillance. Hydrogenera will approach these uses after the civil launch"

**S12.8** Replace "and its own energy management system and can therefore be moved"
With "and its own energy management system. It can therefore be moved"

### Evaluator voice and defensive wording

**S8.7** Replace "Its starting level is therefore set by its least mature element, the airframe at TRL 3, even though most of its subsystems are flight-proven or commercial. The project will raise the maturity of the integrated aircraft task by task: the powertrain will first be validated in the laboratory (Task 2), the assembled aircraft will then be validated on the ground with the fuel cell running (Task 3), and the flight campaign will finally demonstrate it"
With "Its starting level will therefore be TRL 3, the level of its least mature element, the airframe. The project will raise the maturity of the integrated aircraft task by task. The powertrain will first be validated in the laboratory (Task 2), and the assembled aircraft will then be validated on the ground with the fuel cell running (Task 3). The flight campaign will finally demonstrate it"

**S8.8** Delete "In this way every level between TRL 3 and TRL 7 will be reached through its own test stage."

**S8.11** Replace "will be reported against the 90 minute minimum, for which Annex B2 shows the combinations of mass, power and energy that still meet it. These shortfalls are treated as project risks in Section 11, which describes the mitigation measure for each of them and the task in which it will be detected."
With "will be reported against the 90 minute minimum (Annex B2). Section 11 addresses these shortfalls as project risks, each with its mitigation measure and the task in which it will be detected."

**B1.2** Annex B1: replace "Evidence boundary. This annex substantiates" with "Scope. This annex documents"

**B1.3** Annex B1: replace "The proposed project procurement is therefore based on an identified supplier configuration rather than an unconfirmed future component choice."
With "The project procurement is therefore based on an identified and quoted supplier configuration."

**AnxD.1** Annex D: replace "This reference is used here as evidence of aerospace-facing storage manufacture and customer delivery, not as evidence of H2AIR-4+1 aircraft qualification."
With "This reference shows aerospace-facing storage manufacture and customer delivery."

**AnxD.2** Annex D: delete "It is not presented as evidence that the H2AIR-4+1 fuel-cell aircraft or a production aircraft ICE has already been qualified."

### Redundancy and word count

**S7.16** Replace "This is also a new capability, verified from flight logs and the signed checklists of the integration package."
With "It will be verified from flight logs and the signed commissioning checklists."

**S7.17** Replace "Hydrogenera will have held at least five structured customer interviews, obtained at least two written expressions of interest, at least one of them from an owner of linear infrastructure and at least one from outside its existing customer base, and released a cost model with a price-volume curve at 1, 10 and 50 units and the cost per inspected kilometre. The baseline is the prototype bill of materials at one-off prices (Annex B3) with no written customer commitment today, and verification will rest on the signed documents and the cost model (Section 12)."
With "Hydrogenera will have held at least five structured customer interviews and obtained at least two written expressions of interest, one of them from an owner of linear infrastructure. It will also have released a cost model with a price-volume curve and the cost per inspected kilometre (Task 6). The baseline is the one-off prototype bill of materials (Annex B3) with no written customer commitment today, and verification will rest on the signed documents and the cost model."

**S7.18** Replace "Across these KPIs, the 90 minute minimum is the commitment and 120 minutes is the design target, and Annex B2 records which combinations of mass, power and energy still pass the minimum."
With "For these KPIs, Annex B2 records which combinations of mass, power and energy will still pass the 90 minute minimum of KPI 1."

**S7.19 (required)** Delete "A stage will start only after the previous one has passed its written go/no-go gate." Section 8 states the same, and without this cut Section 7 exceeds 2000 words.

**S8.3** Replace "supplier documentation, certification and a current quotation (Annexes A1 to A3 and B1)"
With "supplier documentation, certification, a current quotation and a stock confirmation (Annexes A1 to A3 and B1)"

**S8.6** Replace "at a module mass of 1.45 kg. Hydrogenera holds a current quotation and a stock confirmation for this package (Annexes A1, A2 and B1)."
With "at a module mass of 1.45 kg (Annexes A1, A2 and B1)."

**S8.4** Replace "The maturity map that follows sets out where each element of the aircraft starts and which level it will reach by project end."
With "The maturity map that follows sets out where each element starts and which level it will reach."

**S8.12** Delete "Both levels will build directly on the TRL 7 evidence that this project will produce."

**S12.10** Delete "Task 6 will also deliver the sizing study of the reference corridor, the specification of the refuelling unit, the series manufacturing procedure, the production capacity plan and one documented partnering discussion through the IH-MIE hub."

### Month references

**S10.1** In the six task titles, delete " (M1 to M2)", " (M2 to M5)", " (M4 to M6)", " (M6 to M10)", " (M10 to M11)" and " (M6 to M11)".

### Hygiene: typos, em dashes, internal wording

**S11.1** Delete the duplicated heading "11. Risk Management".

**S11.2** Replace "budget review review against" with "budget review against"

**B1.1** Annex B1: replace "row in in Annex A3" with "row in Annex A3"

**B3.1** Annex B3, Summary C3: replace "using user-supplied quoted prices" with "using supplier-quoted prices"

**B3.3** Annex B3, E18: replace "Latest config carries two boards" with "Design basis: two boards"

**B3.4** Annex B3, E26 and E28: replace "Latest config:" with "Design basis:"

**AnxC.1** In the titles of Annexes C1, C2 and C5, replace the em dash with a colon, for example "HOST UAV: CONTROL & DATA BLOCK DIAGRAM".

**AnxC.3** Annex C2: replace "Payload Power (if required)" with "Payload Power"

**AnxC.4** Annex C5 note: replace "Exact physical positions should be transferred from the released CAD assembly."
With "Exact physical positions are given in the released drawings (Annexes C3 and C4)."

---



# Part M: updates for the new aircraft model run

Figures of the new run: take-off mass 14.0 kg, cruise 18 m/s at 433 W, wing span 3.17 m, mission energy 887 Wh with about 17 Wh from the buffer battery, 60 N required per lift motor. Unchanged: 15-inch pusher propeller, 71 N lift motors, 5 kW peak, 875 Wh usable, 15 kg gate.

## M1. Application draft

Four occurrences of "485 W" stay as they are, because each is the KPI 3 limit: the KPI 3 sentence in S7 (edited in M1.6), "at or below 485 W (KPI 3)" in S8 and in S10 Task 4, and "above 485 W (KPI 3)" in S11.

**M1.1 (S7)** Replace "a wing span of about 3.5 m and a still-air range of about 140 km in two hours"
With "a wing span of about 3.2 m and a still-air range of about 125 km in two hours"

**M1.2 (S7)** Replace "First, the aircraft will need 485 W in cruise at 20 m/s."
With "First, the aircraft will need 433 W in cruise at 18 m/s."

**M1.3 (S7)** Replace "and the buffer battery will supply the remaining 118 Wh of a 120 minute mission"
With "and it will cover the 887 Wh of a 120 minute mission with about 17 Wh from the buffer battery"

**M1.4 (S7)** Replace "four lift motors of at least 62 N each" with "four lift motors of at least 60 N each"

**M1.5 (S7)** Replace "replace the estimated 485 W demand" with "replace the estimated 433 W demand"

**M1.6 (S7, KPI 3)** Replace "Cruise will require at most 485 W, the design value in Annex B2,"
With "Cruise will require at most 485 W, against a design value of 433 W (Annex B2),"

**M1.7 (S10, Task 1)** Replace "a cruise demand of 485 W and the 15 kg MTOW gate" with "a cruise demand of 433 W and the 15 kg MTOW gate"

**M1.8 (S10, Task 2)** Replace "cruise at 485 W, a fuel-cell shutdown" with "cruise at 433 W, a fuel-cell shutdown"

**M1.9 (S10, Task 2)** Replace "each lift motor delivers at least 62 N" with "each lift motor delivers at least 60 N"

**M1.10 (S11)** Replace "its 118 Wh share of the mission" with "its share of the mission"

**M1.11 (S11)** Replace "a thrust below 62 N per motor" with "a thrust below 60 N per motor"

**M1.12 (S11)** Replace "offers 71 N against the 62 N required" with "offers 71 N against the 60 N required"

**M1.13 (S12)** Replace "two hours and more than 100 km of corridor per sortie" with "two hours and about 100 km of corridor per sortie"

The remaining model figures in the draft are already inside S7.7, S7.8, S7.10 and S8.9 of Part A.

## M2. Annex B1

**M2.1** Table header: replace "Mission energy for 120 min. at 485 W cruise" with "Mission energy for 120 min. at 433 W cruise"

**M2.2** Table row: replace "about 118 Wh" with "about 17 Wh" and "993 Wh" with "887 Wh"

**M2.3** Replace "at a cruise electrical demand of 485 W, which the mission model resolves to 993 Wh including vertical flight and transitions, of which 875 Wh comes from hydrogen and about 118 Wh from the buffer battery within its planned discharge to the landing reserve. Hydrogen alone at 485 W cruise gives about 105 minutes."
With "at a cruise electrical demand of 433 W, which the mission model resolves to 887 Wh including vertical flight and transitions, of which 870 Wh comes from hydrogen and about 17 Wh from the buffer battery within its planned discharge to the landing reserve. Hydrogen alone at 433 W cruise gives about 118 minutes."

## M3. Annex B2



### Header

**M3.1** Replace "of 19 September 2026 (configuration hash 9b1305a5)" with "of 20 September 2026 (configuration hash 8e916a8d)"

**M3.2** Replace "and which passes its screening gates except the two marked open in section 6.8" with "and which passes its screening gates except those marked open in section 6.8"

### Section 1

**M3.3** Replace "The cylinder sits about 0.14 m aft" with "The cylinder sits about 0.15 m aft"

**M3.4** Replace "Lift rotors stop in cruise; low-drag stopping, alignment or folding will be selected on the thrust stand (section 5.1)."
With "Lift rotors stop and align with the airflow in cruise and are not folded; the stopped state will be confirmed on the thrust stand (section 5.1)."

### Section 2

**M3.5** Replace "14.4 kg (C, section 6.7), which leaves 0.6 kg to the gate" with "14.0 kg (C, section 6.7), which leaves 1.0 kg to the gate"

**M3.6** Replace "Cruise speed: 20 m/s, about 70 km/h (E)." with "Cruise speed: 18 m/s, about 65 km/h (E)."

**M3.7** Replace "485 W (C, model v4.1 at the closed mass: 450 W for cruise propulsion at 20 m/s plus 35 W auxiliary load"
With "433 W (C, model v4.1 at the closed mass: 398 W for cruise propulsion at 18 m/s plus 35 W auxiliary load"

**M3.8** Replace "993 Wh (C, time-domain mission model), of which 952 Wh in cruise and 41 Wh in pre-flight, vertical flight, transitions and climb; 875 Wh from hydrogen and about 118 Wh from the buffer battery (C: 993 minus 875)"
With "887 Wh (C, time-domain mission model), of which 849 Wh in cruise and 38 Wh in pre-flight, vertical flight, transitions and climb; 870 Wh from hydrogen and about 17 Wh net from the buffer battery, which ends the mission at 93% charge"

**M3.9** Replace "Endurance on hydrogen alone at 485 W cruise: about 105 minutes (C)." with "Endurance on hydrogen alone at 433 W cruise: about 118 minutes (C)."

**M3.10** Replace "(peaks, 118 Wh mission share, landing reserve)" with "(peaks, 17 Wh mission share, landing reserve)"

**M3.11** Replace "at least 62 N (C: 1.5 × 15.5 kg ceiling × 1.08 climb allowance × 9.81 / 4)" with "at least 60 N (C: 1.5 × 15 kg ceiling × 1.08 climb allowance × 9.81 / 4)"

### Section 3

**M3.12** Replace "after 41 Wh for pre-flight" with "after 38 Wh for pre-flight"

**M3.13** Replace the three rows with:

- "Hydrogen only at 15% reserve (875 Wh): 433 W, 118 min; 485 W, 106 min; 550 W, 94 min; 572 W, 90 min."
- "With the planned battery contribution of 17 Wh (893 Wh): 433 W, 121 min; 500 W, 105 min; 550 W, 95 min; 584 W, 90 min."
- "Hydrogen only at 20% reserve (824 Wh): 433 W, 111 min; 500 W, 97 min; 537 W, 90 min."

**M3.14** Replace the "Reading:" paragraph up to "on hydrogen alone." with:
"Reading: at the design demand of 433 W the 120 minute target is met with the full cylinder, the 15% reserve and about 17 Wh from the buffer battery, and hydrogen alone gives 118 minutes. At the KPI 3 ceiling of 485 W, hydrogen alone still gives 106 minutes. The 90 minute minimum (KPI 1) holds on hydrogen alone up to a cruise demand of about 572 W, one third above the design value, and up to about 537 W with a 20% reserve."
Keep the last two sentences of that paragraph.

### Section 5.1

**M3.15** Replace "at least 62 N static thrust each at the 15.5 kg design ceiling; one pusher motor and propeller for 20 m/s wing-borne cruise. Low-drag stopping, alignment or folding of the lift propellers will be evaluated on the thrust stand."
With "at least 60 N static thrust each at the 15 kg ceiling; one pusher motor and propeller for 18 m/s wing-borne cruise. The stopped and aligned state of the lift propellers will be measured on the thrust stand."

### Section 6.1

**M3.16** Replace "Wing: span 3.52 m, area 0.92 m2, aspect ratio 13.5, mean aerodynamic chord 0.27 m, root chord 0.33 m, taper ratio 0.6, NACA 4412 section with 2 degrees of washout, no sweep or dihedral, main spar 24 x 18 mm carbon-fibre tube (C). Horizontal tail: span 0.83 m, area 0.15 m2, NACA 0012; twin fins 0.33 m high, NACA 0012 (C). Fuselage: 0.86 m long"
With "Wing: span 3.17 m, area 0.98 m2, aspect ratio 10.3, mean aerodynamic chord 0.31 m, root chord 0.36 m, taper ratio 0.71, NACA 6415 section with 2 degrees of washout, no sweep or dihedral, main spar 28 x 26 mm carbon-fibre tube (C). Horizontal tail: span 0.78 m, area 0.13 m2, NACA 0012; twin fins 0.43 m high, NACA 0012 (C). Fuselage: 0.88 m long"

**M3.17** Replace "Booms: 22 x 18 mm carbon-fibre tube, 0.91 m each" with "Booms: 26 x 24 mm carbon-fibre tube, 0.99 m each"

### Section 6.2

**M3.18** Replace "Cruise speed 20 m/s (E)." with "Cruise speed 18 m/s (E)."

**M3.19** Replace "about 140 km (C); corridor planning value more than 100 km per sortie" with "about 125 km (C); corridor planning value about 100 km per sortie"

**M3.20** Replace "Cruise drag 16.2 N and lift-to-drag ratio 8.7 at the closed mass, with the lift propellers stopped and aligned and the landing legs fixed (C). Wing loading 15.7 kg/m2 and stall speed 14.7 m/s at the closed mass, 15.0 m/s at 15 kg, with a maximum lift coefficient of 1.19 (C)"
With "Cruise drag 13.4 N and lift-to-drag ratio 10.2 at the closed mass, with the lift propellers stopped and aligned (C). Wing loading 14.3 kg/m2 and stall speed 13.1 m/s at the closed mass, 13.5 m/s at 15 kg, with a maximum lift coefficient of 1.37 (C)"
The cruise-to-stall ratio of 1.33 at 15 kg is unchanged.

**M3.21** Replace "Forward and reverse transitions of about 18 s each, using 8.6 Wh and 4.8 Wh" with "Forward and reverse transitions of about 15 s and 20 s, using 6.9 Wh and 4.2 Wh"

### Section 6.3

**M3.22** Replace "a thrust-to-weight ratio of 2.0 at the closed mass" with "a thrust-to-weight ratio of 2.1 at the closed mass"

**M3.23** Replace "the front motors need 66 N and the rear motors 57 N against the 71 N available, because the centre of gravity lies 22 mm ahead of the centre of the lift rows"
With "the front motors need 51 N and the rear motors 68 N against the 71 N available, because the centre of gravity lies 43 mm aft of the centre of the lift rows"

**M3.24** Replace "against a cruise propulsion demand of 450 W (C)" with "against a cruise propulsion demand of 398 W (C)"

### Section 6.4

**M3.25** Replace "delivers 875 Wh from 52.6 g of hydrogen and 9.3 g remain" with "delivers 870 Wh from 52.3 g of hydrogen and 9.6 g remain"

### Section 6.6

**M3.26** Replace "Centre of gravity at 0.454 m from the nose and neutral point at 0.487 m, which gives a static margin of 12.6%"
With "Centre of gravity at 0.443 m from the nose and neutral point at 0.492 m, which gives a static margin of 15.7%"

**M3.27** Replace "The six-degree-of-freedom screening of the closed layout passes its gates for trim, transition attitude and gust response with linear aerodynamic derivatives, which remain"
With "The six-degree-of-freedom screening of the closed layout uses linear aerodynamic derivatives and is open at this run (section 6.8); the derivatives remain"

### Section 6.7

**M3.28** Replace "Closed take-off mass 14.37 kg (C): wing structure 3.38 kg, tail structure 0.86 kg, fuselage structure 1.34 kg, booms, mounts and cross-member 0.47 kg, landing gear 0.06 kg, propulsion 1.33 kg, energy system 5.08 kg, avionics 0.64 kg and payload 1.20 kg."
With "Closed take-off mass 14.00 kg (C): wing structure 2.62 kg, tail structure 0.81 kg, fuselage structure 0.99 kg, booms, mounts and cross-member 0.49 kg, propulsion 1.34 kg, energy system 5.34 kg, avionics, wiring and servos 1.21 kg and payload 1.20 kg."

**M3.29** Replace "the fuel-cell stack section 1.09 kg and control box 0.36 kg (S, 1.45 kg module), the regulator 0.32 kg (S), the hydrogen 0.06 kg and a buffer battery of 1.63 kg"
With "the fuel-cell stack section 1.10 kg and control box 0.36 kg (S, 1.45 kg module), the regulator 0.32 kg (S), the hydrogen 0.06 kg, hydrogen plumbing and mounts 0.21 kg (E) and a buffer battery of 1.65 kg"

**M3.30** Replace "(wing shell 2.2 kg, horizontal tail shell 0.5 kg, fin shells 0.25 kg, fuselage shells 1.19 kg)" with "(wing shell 1.76 kg, horizontal tail shell 0.24 kg, fin shells 0.20 kg, fuselage shells 0.79 kg)"

### Section 6.8

**M3.31** Replace "transition, longitudinal stability, six-degree-of-freedom control, lift thrust at 15 kg and the structural member screen. Two gates are open because they need data that the model does not hold:"
With "transition, longitudinal stability, tail control authority, lift thrust at 15 kg and the structural member screen. The two six-degree-of-freedom control gates are open at this run and will be closed with the geometry update of the Task 1 design freeze. Two further gates are open because they need data that the model does not hold:"
The run reports both six-degree-of-freedom gates as failed, so they are written as open.

### Section 7

**M3.32** KPI 3: replace "Pass: 485 W or less." with "Pass: 485 W or less, against a design value of 433 W."

**M3.33** KPI 4: replace "Pass: at least 62 N per motor" with "Pass: at least 60 N per motor"

## M4. Annex B3

**M4.1** D11 and E11: replace "22 mm boom" with "26 mm boom" (two cells).

**M4.2** E26: replace "~0.8594 m each" with "~0.88 m each"

**M4.3** E27: replace "Current design section 24 mm OD / 18 mm ID over about 3.5 m span." with "Design section 28 mm OD / 26 mm ID over about 3.2 m span."
K27: replace "24 x 18 mm CFRP" with "28 x 26 mm CFRP"

**M4.4** E29: replace "Latest hardcoded section 22 mm OD / 18 mm ID; each boom ~1.00 m between current stations" with "Design section 26 mm OD / 24 mm ID; each boom about 0.99 m (Annex B2)"
K29: replace "22 x 18 x 2000 mm" with "26 x 24 x 2000 mm"

**M4.5** Check the prices of rows 11, 27 and 29 for the new tube sizes. If the total moves, update "about €24,700" and "about €17,800" in S12.

Annexes A, C, D, E and F carry no model figures.

---



# Part P: internal to-dos and placeholders



## P1. Delete now (application draft)

**P1.1** The header line "Application package synchronized ... 15 September 2026".

**P1.2** The six blocks that start with:

- "INTERNAL MANUFACTURING TO-DO"
- "INTERNAL SIZING AND CONFIGURATION TO-DO"
- "INTERNAL PERFORMANCE TO-DO"
- "INTERNAL HOST-EVIDENCE TO-DO" followed by "Photos/video"
- "INTERNAL HOST-EVIDENCE TO-DO" followed by "Test-site record"
- "INTERNAL GENDER-EVIDENCE TO-DO"

**P1.3** The placeholder "[NEEDED FROM HYDROGENERA: measured minutes]" in S9 is removed by S9.1 of Part A.

## P2. Fill in (application draft)

**P2.1 (S12)** Replace "will fly about three to four hours [NEEDED FROM HYDROGENERA: confirm with the sizing model]"
With "will fly about four to five hours"
This is a projection from the new run with the same wing and speed: about 4.2 h at 16.4 kg with the 6.8 L cylinder and about 5.4 h at 16.8 kg with the 9.0 L cylinder. Confirm it with a model run before use, because the stall margin at those masses falls below the 1.3 gate.

**P2.2 (S8)** Put " for [X] minutes of airborne time" after "and it held a stable, controlled hover". Put ", during a flight of [Y] minutes" after "with a mean horizontal error of 0.22 m".

**P2.3** Portal fields: exact female and total headcount, female-led status and LDR status.

## P3. Fill in (annexes)

**P3.1 Annex B2, section 1.** "stands on a tripod of one nose leg and two wing legs": the new run has no landing legs and no landing-gear mass. State how the aircraft will stand, then replace this sentence.

**P3.2 Annex B2 (sections 2, 5.4 and 6.7), Annex B1 and Annex B3 row 3.** Enter the buffer battery capacity, cell count, peak current and mass from the fuel-cell supplier. The 1.65 kg in B2 is a model allowance. If the delivered mass differs, recompute the 165, 260 and 325 Wh/kg figures in S7.10 and B2.2.

**P3.3 Annex C6.** Add a row "Airborne time (min)" to the Evidence Summary sheet.

**P3.4 Annex C7.** The file is empty. Add dated aircraft photos, an equipment photo, the aircraft on the scale, the two captioned videos and one line per flight describing the test site.

**P3.5 Annexes C3 and C4.** Fill the "Checked" and "Approved" boxes. Add a note that the drawings issued on 12.09.2026 record the as-flown configuration of 23 July and 25 August 2026.

**P3.6 Annex D.** Replace every "HYDROGENERA INPUT REQUIRED" cell: the eight team rows (name and allocation), the three rows of the EU-project record table, and the bracketed sentence on the measured status of the metal-hydride compressor.

**P3.7 Annex F.** F15: confirm the 53 kWh/kg figure and its source document. F20: insert the call link.

---



# Part DEC: decisions and inputs still needed

**DEC.1 Reference battery pack.** Confirm that you will buy and fly a pack of the same mass as the hydrogen energy system, about 5.3 kg. A conventional lithium-polymer pack should give roughly 90 minutes (estimate). A semi-solid-state pack of that mass could match the hydrogen result, so state the chemistry once chosen.

**DEC.2 Bench run with a 6.8 L cylinder (optional, one extra cylinder).** If accepted, put this in the Task 2 description after the thrust-stand sentence: "The bench will finally repeat the cruise profile on a 6.8 L cylinder, so that the growth of endurance with cylinder size will be measured on the same powertrain."

**DEC.3 S12 pricing.** Confirm whether the €55,000 to €65,000 price excludes the inspection payload. If it does, put ", likewise without payload," after "per system". Confirm the basis of the 55 to 65% margin, since materials alone give 60 to 69%.

**DEC.4 Budget field (10 points, not drafted).** Needed: person-months and rates per task, the spares list, and whether the payload is bought. The text will then be written against the €24,700 of Annex B3 and the €60,000 lump sum.

**DEC.5 LDR status of the registered seat.** This is 10 automatic points and decides the Impact minimum.

**DEC.6 Digital hub paragraph (S9.3).** Check the Guide's wording for the +2 bonus and whether the portal has its own field for it.

**Closed decision.** KPI 3 stays at "at most 485 W" with 433 W as the design value (M1.6, M3.14, M3.32).