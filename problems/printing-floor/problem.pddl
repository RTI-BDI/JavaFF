( define ( problem printing1 )
( :domain printing-domain )
    ( :objects
        r1 r2 r3 - robot
        p1 p2 - printer
        r_a r_b r_c r_d r_e r_f - room
        h11 h12 h13 h21 h22 h23 h31 h32 h33 - hallway_segment
        h11_21 h13_23 h21_31 h23_33 - hallway_segment
        d0 d1 d2 d3 - dock
    )
    ( :init
        (near r_a h11)

        (near h11 h12)
        (near h11 h11_21)
        (near h11 r_a)

        (near h12 h11)
        (near h12 h13)

        (near h13 h12)
        (near h13 r_d)
        (near h13 h13_23)

        (near r_d h13)

        (near r_b h11_21)

        (near h11_21 h11)
        (near h11_21 h21)
        (near h11_21 r_b)

        (near h13_23 h13)
        (near h13_23 h23)
        (near h13_23 r_e)

        (near r_e h13_23)

        (near h21 h11_21)
        (near h21 h22)
        (near h21 h21_31)
        
        (near h22 h21)
        (near h22 h23)

        (near h23 h13_23)
        (near h23 h22)
        (near h23 h23_33)

        (near r_c h21_31)

        (near h21_31 r_c)
        (near h21_31 h21)
        (near h21_31 h31)

        (near h23_33 h23)
        (near h23_33 h33)
        (near h23_33 r_e)

        (near r_e h23_33)

        (near h31 h21_31)
        (near h31 h32)

        (near h32 h31)
        (near h32 h33)

        (near h33 h23_33)
        (near h33 h32)
        (near h33 r_f)

        (near r_f h33)

        (free r_a)
        (free r_b)
        (free r_c)
        (free r_d)
        (free r_e)
        (free r_f)

        (free h11)
        (free h12)

        (free h11_21)
        (free h13_23)

        (free h21)
        (free h23)

        (free h21_31)
        (free h23_33)

        (free h31)
        (free h32)
        (free h33)

        (p_in p1 h12)
        (p_in p2 h32)

        (available p1)

        (d_in d0 r_c)
        (d_in d1 r_c)
        (d_in d2 r_c)
        (d_in d3 r_c)

        (r_in r1 r_d)
        (inactive r1)
        (not_r_docked r1)
        
        (r_in r2 r_c)
        (inactive r2)
        (r_docked r2)
        
        (r_in r3 r_c)
        (inactive r3)
        (r_docked r3)
        
        (= (battery_charge r1) 30)
        (= (battery_charge r2) 100)
        (= (battery_charge r3) 100)
    )
    ( :goal
        ( and
            ;; (r_in r2 h21_31)
            (printed_docs_left_in r2 r_e)
        )
    )
)