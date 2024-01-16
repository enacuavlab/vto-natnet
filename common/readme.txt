https://optitrack.com/support/downloads/developer-tools.html

Download for windows
https://s3.amazonaws.com/naturalpoint/software/NatNetSDK/NatNet_SDK_4.1.zip 

Get NatNetSDK/Samples/PythonClient
- DataDescriptions.py
- MoCapData.py
- NatNetClient.py

Patch NatNetClient.py
"
  self.rigid_body_listener = None
  self.new_frame_listener  = None
  self.rigid_body_list_listener = None
"
"
  self.new_frame_listener( data_dict )
  if self.rigid_body_list_listener is not None:
    self.rigid_body_list_listener(rigid_body_data, timestamp)
"

./natnet41.py

#./natnet2ivy.py -xs right -up z_up -le near -an far -ac 217 217 -s 192.168.1.240
