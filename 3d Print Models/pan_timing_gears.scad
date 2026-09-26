$fn = 96;

/* ========================================================================
   HTD-3M TIMING BELT DRIVE SET (TURNTABLE RING & NEMA 17 PINION)
   ========================================================================
   - Ring:           246T HTD-3M with dual flanges
   - Pinion:         37T HTD-3M with dual flanges (Ratio: 6.65:1)
   - 2020 Interface: 4mm Riser Pad + 1.5mm Alignment Tongue for 2020 T-Slot
   - Fasteners:      4x M6 Heat-Set Inserts (Bottom), 4x M5 Through-Slots (Top)
   - Belt Spec:      HTD-3M, 15mm Width (Channel = 17mm)
   - Recommended Belt: 840-3M-15 (~162.7mm center-to-center distance)
   ======================================================================== */

/* --- VIEW / PRINT MODE --- */
// Options: "assembly", "ring", "pinion"
mode = "assembly"; 

/* --- COMMON TIMING BELT PARAMETERS --- */
pitch           = 3.0;   // HTD-3M 3mm tooth pitch
belt_w          = 15.0;  // 15mm belt width
face_w          = 17.0;  // 17mm belt channel (+2mm clearance)
flange_chamf_h  = 1.0;   // 45° chamfer height (mm)
flange_rim_h    = 1.0;   // Vertical blunt flat edge height (mm)
flange_total_h  = flange_chamf_h + flange_rim_h; // 3.0mm total flange height
u_pitch_offset  = 0.38;  // Distance from pitch line to outer tooth crown
tooth_depth     = 1.17;  // HTD-3M groove depth
tooth_radius    = 0.74;  // Curvilinear tooth trough radius

/* --- 246T RING SPECIFICATIONS --- */
ring_teeth      = 246;
ring_pitch_r    = (ring_teeth * pitch) / (2 * PI); // ~117.46mm (Dia ~234.92mm)
ring_outer_r    = ring_pitch_r - u_pitch_offset;   // ~117.08mm
ring_flange_r   = ring_outer_r + flange_chamf_h;   // 45° radial bevel (~119.08mm)
bore_d          = 140.0;                           // Center clearance bore (mm)

// 2020 Extrusion Riser & Alignment Tongue
riser_h         = 4.0;                             // 4mm riser pad clearance height
tongue_w        = 5.8;                             // Slot tongue width (clears standard 6mm slot)
tongue_h        = 1.5;                             // Slot tongue protrusion height

// Lazy Susan 136mm Square Pattern (Radius = 136 / sqrt(2) ≈ 96.167mm)
ls_hole_pitch      = 136.0;
mount_radius       = ls_hole_pitch / sqrt(2);
m6_insert_hole_dia = 7.30;                         // M6 heat-set insert pocket diameter (mm)
m6_insert_depth    = 12.50;                        // M6 insert pocket depth (mm)

// Top Face M5 2020 T-Slot Fasteners
m5_clearance_d     = 5.5;                          // M5 shank clearance hole (mm)
m5_head_cbore_d    = 10.0;                         // M5 counterbore diameter (mm)
m5_shelf_thick     = 5.5;                          // Clamping shelf thickness at top face (mm)

// Counterbore depth measured from the bottom of the lower flange up toward the flat 4mm shelf:
total_riser_top    = face_w + 2 * flange_total_h + riser_h;
m5_head_recess     = total_riser_top - m5_shelf_thick;

/* --- 37T PINION SPECIFICATIONS (NEMA 17) --- */
pinion_teeth       = 37;                           // 246 / 37 = 6.649:1
pinion_pitch_r     = (pinion_teeth * pitch) / (2 * PI); // ~17.67mm (Dia ~35.33mm)
pinion_outer_r     = pinion_pitch_r - u_pitch_offset;   // ~17.29mm
pinion_flange_r    = pinion_outer_r + flange_chamf_h;   // 45° radial bevel (~19.29mm)
nema_shaft_dia     = 5.1;                          // 5mm shaft (+0.2mm 3D print clearance)
nema_flat_dist     = 4.6;                          // D-flat distance from rounded back (mm)

// Pinion Fasteners
m3_screw_dia       = 3.2;                          // M3 set-screw clearance (mm)
m3_cbore_dia       = 6.5;                          // Counterbore for M3 screw head (mm)
m3_head_height     = 3.5;                          // Set-screw recess depth (mm)
pinion_valley_ang  = 360.0 / (4 * pinion_teeth);

/* --- CENTER DISTANCE FOR 810mm BELT (270 TEETH) --- */
center_distance    = 162.1; 


/* ========================================================================
   MAIN DISPLAY SWITCH
   ======================================================================== */

if (mode == "assembly") {
    // Render Ring Pulley at origin
    htd3m_ring();
    
    // Render NEMA 17 Pinion aligned along +X axis
    translate([center_distance, 0, 0])
        htd3m_pinion();

} else if (mode == "ring") {
    htd3m_ring();

} else if (mode == "pinion") {
    htd3m_pinion();
}


/* ========================================================================
   MODULE DEFINITIONS
   ======================================================================== */

// 1. LAZY SUSAN HTD-3M 246T RING PULLEY
module htd3m_ring() {
    base_flange_top = face_w + flange_total_h;
    pad_radial_len  = (ring_outer_r - (bore_d / 2.0)) + 1.5;

    difference() {
        union() {
            // Main tooth face cylinder
            cylinder(r = ring_outer_r, h = face_w);

            // --- Lower Flange (Blunt Vertical Rim + 45° Chamfer) ---
            // Blunt vertical bottom edge sitting flat on bed
            translate([0, 0, -flange_total_h])
                cylinder(r = ring_flange_r, h = flange_rim_h);
            // 45° upward taper expanding into tooth channel
            translate([0, 0, -flange_chamf_h])
                cylinder(r1 = ring_flange_r, r2 = ring_outer_r, h = flange_chamf_h);

            // --- Upper Flange (45° Chamfer + Blunt Vertical Top Rim) ---
            // 45° outward slope
            translate([0, 0, face_w])
                cylinder(r1 = ring_outer_r, r2 = ring_flange_r, h = flange_chamf_h);
            // Blunt vertical rim cap
            translate([0, 0, face_w + flange_chamf_h])
                cylinder(r = ring_flange_r, h = flange_rim_h);

            // --- 2020 Extrusion Riser & Tongue ---
            for (angle = [0, 180]) {
                rotate([0, 0, angle]) {
                    // 1. Flat 4mm Riser Pad (20mm wide to match 2020 bar)
                    translate([bore_d / 2.0 - 1.0, -10.0, base_flange_top])
                        cube([pad_radial_len, 20.0, riser_h]);

                    // 2. 1.5mm Alignment Tongue (Centered on 2020 slot)
                    translate([bore_d / 2.0 - 1.0, -tongue_w / 2.0, base_flange_top + riser_h])
                        cube([pad_radial_len, tongue_w, tongue_h]);
                }
            }
        }

        // ================= SUBTRACTIONS =================

        // 1. HTD-3M Curvilinear Tooth Pockets
        for (i = [0 : ring_teeth - 1]) {
            rotate([0, 0, i * (360 / ring_teeth)]) {
                translate([ring_pitch_r, 0, -0.0]) {
                    linear_extrude(height = face_w + 0.0) {
                        hull() {
                            circle(r = tooth_radius, $fn = 24);
                            translate([-tooth_depth, 0, 0])
                                circle(r = tooth_radius * 0.75, $fn = 24);
                        }
                    }
                }
            }
        }

        // 2. Central Clearance Bore
        translate([0, 0, -flange_total_h - 5])
            cylinder(d = bore_d, h = total_riser_top + tongue_h + 10);

        // 3. Bottom Face: 4x M6 Heat-Set Inserts for 136mm Lazy Susan
        for (a = [45, 135, 225, 315]) {
            rotate([0, 0, a]) {
                translate([mount_radius, 0, -flange_total_h - 0.1])
                    cylinder(d = m6_insert_hole_dia, h = m6_insert_depth + 0.1);
            }
        }

        // 4. Top/Through: M5 Counterbored Fasteners for 2020 T-Slot (0° and 180°)
        for (angle = [0, 180]) {
            rotate([0, 0, angle]) {
                // Outer hole: mount_radius + 10mm (~106.17mm)
                translate([mount_radius + 10, 0, 0]) {
                    // Full M5 Shank Clearance Through-Hole
                    translate([0, 0, -flange_total_h - 1])
                        cylinder(d = m5_clearance_d, h = total_riser_top + tongue_h + 2);
                    // Counterbore from bottom up to (total_riser_top - m5_shelf_thick)
                    translate([0, 0, -flange_total_h - 0.1])
                        cylinder(d = m5_head_cbore_d, h = m5_head_recess + 0.1);
                }

                // Inner hole: mount_radius - 15mm (~81.17mm)
                translate([mount_radius - 15, 0, 0]) {
                    // Full M5 Shank Clearance Through-Hole
                    translate([0, 0, -flange_total_h - 1])
                        cylinder(d = m5_clearance_d, h = total_riser_top + tongue_h + 2);
                    // Counterbore from bottom up to (total_riser_top - m5_shelf_thick)
                    translate([0, 0, -flange_total_h - 0.1])
                        cylinder(d = m5_head_cbore_d, h = m5_head_recess + 0.1);
                }
            }
        }
    }
}

// 2. NEMA 17 STEPPER 37T PINION (D-SHAFT & RADIAL SET-SCREW RECESSED COUNTERBORE)
module htd3m_pinion() {
    total_h        = face_w + 2 * flange_total_h;
    pinion_root_r  = pinion_pitch_r - tooth_depth;
    pinion_shelf_r = pinion_root_r - m3_head_height;
    flat_x_pos     = nema_flat_dist - (nema_shaft_dia / 2.0);

    difference() {
        union() {
            // Main tooth cylinder
            cylinder(r = pinion_outer_r, h = face_w);

            // Lower Flange (Blunt Vertical Rim + 45° Chamfer)
            translate([0, 0, -flange_total_h])
                cylinder(r = pinion_flange_r, h = flange_rim_h);
            translate([0, 0, -flange_chamf_h])
                cylinder(r1 = pinion_flange_r, r2 = pinion_outer_r, h = flange_chamf_h);

            // Upper Flange (45° Chamfer + Blunt Vertical Top Rim)
            translate([0, 0, face_w])
                cylinder(r1 = pinion_outer_r, r2 = pinion_flange_r, h = flange_chamf_h);
            translate([0, 0, face_w + flange_chamf_h])
                cylinder(r = pinion_flange_r, h = flange_rim_h);
        }

        // ================= SUBTRACTIONS =================

        // 1. HTD-3M Curvilinear Tooth Pockets
        for (i = [0 : pinion_teeth - 1]) {
            rotate([0, 0, i * (360 / pinion_teeth)]) {
                translate([pinion_pitch_r, 0, -0.0]) {
                    linear_extrude(height = face_w + 0.0) {
                        hull() {
                            circle(r = tooth_radius, $fn = 20);
                            translate([-tooth_depth, 0, 0])
                                circle(r = tooth_radius * 0.75, $fn = 20);
                        }
                    }
                }
            }
        }

        // 2. D-Shaft Bore (Through entire height)
        rotate([0, 0, pinion_valley_ang])
            translate([0, 0, -flange_total_h - 1])
                d_shaft_bore(h = total_h + 2);

        // 3. M3 Recessed Counterbore Set-Screw Pocket (Aligned to D-flat at mid-height)
        rotate([0, 0, pinion_valley_ang]) {
            translate([0, 0, face_w / 2.0]) {
                // Shank clearance hole: D-flat out to shelf
                rotate([0, 90, 0])
                    translate([0, 0, flat_x_pos])
                        cylinder(d = m3_screw_dia, h = pinion_shelf_r - flat_x_pos + 0.1);

                // Counterbore pocket: shelf out past outer diameter
                rotate([0, 90, 0])
                    translate([0, 0, pinion_shelf_r])
                        cylinder(d = m3_cbore_dia, h = (pinion_flange_r - pinion_shelf_r) + 5);
            }
        }
    }
}

// Helper: Standard NEMA 17 D-Shaft Cutout (Flat centered on +X)
module d_shaft_bore(h) {
    flat_x = nema_flat_dist - (nema_shaft_dia / 2.0);
    difference() {
        cylinder(d = nema_shaft_dia, h = h);
        translate([flat_x + (nema_shaft_dia / 2.0), 0, h / 2.0])
            cube([nema_shaft_dia, nema_shaft_dia * 2, h + 0.2], center = true);
    }
}