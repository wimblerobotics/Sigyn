 #!/bin/bash
   # shutdown_robots.sh
   
   echo "Shutting down robot computers..."
   
   # Shutdown robot computer
   ssh -i ~/.ssh/id_ed25519 ros@sigynVision.local 'sudo shutdown -h now' &
   
   # Shutdown gripper computer
   ssh -i ~/.ssh/id_ed25519 ros@sigyn7900.local 'sudo shutdown -h now' &
   
   echo "Shutdown commands sent to all robot computers"
   wait
