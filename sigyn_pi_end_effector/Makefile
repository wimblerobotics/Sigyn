
export PATH:=$(PATH):.
LIBCHK="libs.sh"

all: pi_end_effector


pi_end_effector: 
	# libs.chk
	gcc main.cpp -lwiringPi -o pi_end_effector

# libs.chk: -lwiringPiPca9685 
# 	@chmod 755 libs.sh
# 	@$(LIBCHK)
# 	@touch libs.chk

# install: pca9685
# 	@echo installing pca9865 to /usr/local/bin
# 	@sudo cp pca9685 /usr/local/bin

# clean:
# 	@rm -f pca9685 libs.chk