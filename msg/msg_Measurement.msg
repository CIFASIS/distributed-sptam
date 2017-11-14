#msg_Measurement 
int32 source                # SRC_TRIANGULATION=0, SRC_TRACKER=1, SRC_REFIND=2 
int32 type                  # STEREO=0, LEFT=1, RIGHT=2 
int64 id_kf                     
int64 id_mp                     
msg_kp coord_feat_left		# Solo utilizo (x,y) 
msg_kp coord_feat_right		# Solo utilizo (x,y)