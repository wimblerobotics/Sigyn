#!/bin/bash
m4 <wiring_all.yml >foo.yml;wireviz --format hp -O sigyn_all__wiring foo.yml
m4 <wiring_24v_dc_dc.yml >foo.yml;wireviz --format hp -O sigyn_24v_dc_dc__wiring foo.yml
m4 <wiring_12v_dc_dc.yml >foo.yml;wireviz --format hp -O sigyn_12v_dc_dc__wiring foo.yml
m4 <wiring_5v_dc_dc.yml >foo.yml;wireviz --format hp -O sigyn_5v_dc_dc__wiring foo.yml
m4 <wiring_3r3v_dc_dc.yml >foo.yml;wireviz --format hp -O sigyn_3r3v_dc_dc__wiring foo.yml
