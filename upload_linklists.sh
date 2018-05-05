#! /bin/bash

<<<<<<< HEAD
sudo rsync -avz --ignore-existing --delete blast_config/linklists/ /data/etc/linklists/
rsync -avz --ignore-existing --delete blast_config/linklists/ fc1:/data/etc/linklists/
rsync -avz --ignore-existing --delete blast_config/linklists/ fc2:/data/etc/linklists/
#rsync -avz --delete blast_config/linklists fc1user@fc1mustang:/data/etc/
=======
sudo rsync -rtvu --delete blast_config/linklists /data/etc/
rsync -rtvu --delete blast_config/linklists fc1:/data/etc/
rsync -rtvu --delete blast_config/linklists fc2:/data/etc/
#rsync -rtvu --delete blast_config/linklists fc1user@fc1mustang:/data/etc/
>>>>>>> master

