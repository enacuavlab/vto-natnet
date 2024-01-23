These are python script provide by optitrack, to use natnet SDK 
Following patch (*) should be applied to retrieve rigid body and marker set

https://optitrack.com/support/downloads/developer-tools.html

Download for windows
https://s3.amazonaws.com/naturalpoint/software/NatNetSDK/NatNet_SDK_4.1.zip 

Get NatNetSDK/Samples/PythonClient
- DataDescriptions.py
- MoCapData.py
- NatNetClient.py

--------------------------------------------------------------------------------
(*) Patch NatNetClient.py
"
  self.rigid_body_listener = None
  self.new_frame_listener  = None

  self.rigid_body_marker_set_list_listener = None
"
"
  self.new_frame_listener( data_dict )

  if self.rigid_body_marker_set_list_listener is not None:
    self.rigid_body_marker_set_list_listener(rigid_body_data, marker_set_data, timestamp)
"

--------------------------------------------------------------------------------
sudo apt-get install python3-pip
sudo pip3 install pyquaternion
