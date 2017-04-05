MODULE MainModule
	PROC main()
		Heat;
		SetDO DO1_Auto_Mode, 1;

		

		! Part number: 1
		! PartParams(extrusionFactor=0.98, printSpeed=30, shiftX=0.0, shiftY=0.0, shiftZ=0.0, numLayers=4, designType=0, brims=0, horizontalExpansion=0.0, randomStartLocation=1)

		! Layer: 1
		! LayerParams(infillAngleDegrees=0.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T11
		! M6
		TPWRITE "Layer 1 of 4";
		MoveL Offs(pZeroMute, 0.0, 22.0, 5.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 19.053, -11.0, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 18.187, -10.5, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.433, 19.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -0.433, 19.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -1.299, 17.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 1.299, 17.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 2.165, 16.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -2.165, 16.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -3.031, 14.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 3.031, 14.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 3.897, 13.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -3.897, 13.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -4.763, 11.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 4.763, 11.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 5.629, 10.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -5.629, 10.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -6.495, 8.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 6.495, 8.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 7.361, 7.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -7.361, 7.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -8.227, 5.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 8.227, 5.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 9.093, 4.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -9.093, 4.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -9.959, 2.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 9.959, 2.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 10.825, 1.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -10.825, 1.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -11.692, -0.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 11.692, -0.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 12.558, -1.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -12.558, -1.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -13.424, -3.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 13.424, -3.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 14.29, -4.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -14.29, -4.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -15.156, -6.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 15.156, -6.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 16.022, -7.75, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -16.022, -7.75, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -16.888, -9.25, 0.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 16.888, -9.25, 0.2), v30, z0, tNozzle, \Wobj := wobjPlatform;
		SetDO DO8_Triple_Retract, 1;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;        
        WaitTime 0.15;
		MoveL Offs(pZeroMute, 16.888, -9.25, 5.2), v100, z0, tNozzle, \Wobj := wobjPlatform;
        MoveL Offs(pZeroMute, 0, 0, 50), v100, fine \Inpos := inpos50, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO8_Triple_Retract, 0;

		! Layer: 2
		! LayerParams(infillAngleDegrees=60.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T12
		! M6
		TPWRITE "Layer 2 of 4";
		MoveL Offs(pZeroMute, -19.053, -11.0, 5.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
        WaitTime 0.1;
		MoveL Offs(pZeroMute, 19.053, -11.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 18.187, -10.5, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -16.455, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 0.433, 19.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 1.299, 17.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -14.723, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -12.991, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 2.165, 16.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 3.031, 14.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -11.259, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -9.526, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 3.897, 13.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 4.763, 11.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -7.794, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -6.062, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 5.629, 10.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 6.495, 8.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -4.33, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -2.598, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 7.361, 7.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 8.227, 5.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -0.866, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.866, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 9.093, 4.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 9.959, 2.75, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 2.598, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 4.33, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 10.825, 1.25, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 11.691, -0.25, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 6.062, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 7.794, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 12.557, -1.75, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 13.423, -3.25, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 9.526, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 11.258, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 14.29, -4.75, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 15.156, -6.25, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 12.99, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 14.722, -10.0, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 16.022, -7.75, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 16.888, -9.25, 0.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 16.454, -10.0, 0.4), v30, z0, tNozzle, \Wobj := wobjPlatform;
		SetDO DO8_Triple_Retract, 1;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;        
        WaitTime 0.15;
		MoveL Offs(pZeroMute, 16.454, -10.0, 5.4), v100, z0, tNozzle, \Wobj := wobjPlatform;
        MoveL Offs(pZeroMute, 0, 0, 50), v100, fine \Inpos := inpos50, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO8_Triple_Retract, 0;

		! Layer: 3
		! LayerParams(infillAngleDegrees=-60.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T13
		! M6
		TPWRITE "Layer 3 of 4";
		MoveL Offs(pZeroMute, -19.053, -11.0, 5.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
        WaitTime 0.1;
		MoveL Offs(pZeroMute, 19.053, -11.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 18.187, -10.5, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -16.454, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -16.888, -9.25, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -16.022, -7.75, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -14.722, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -12.99, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -15.156, -6.25, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -14.29, -4.75, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -11.258, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -9.526, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -13.423, -3.25, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -12.557, -1.75, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -7.794, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -6.062, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -11.691, -0.25, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -10.825, 1.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -4.33, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -2.598, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -9.959, 2.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -9.093, 4.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -0.866, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.866, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -8.227, 5.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -7.361, 7.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 2.598, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 4.33, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -6.495, 8.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -5.629, 10.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 6.062, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 7.794, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -4.763, 11.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -3.897, 13.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 9.526, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 11.259, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -3.031, 14.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -2.165, 16.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 12.991, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 14.723, -10.0, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -1.299, 17.75, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -0.433, 19.25, 0.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 16.455, -10.0, 0.6), v30, z0, tNozzle, \Wobj := wobjPlatform;
		SetDO DO8_Triple_Retract, 1;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;        
        WaitTime 0.15;
		MoveL Offs(pZeroMute, 16.455, -10.0, 5.6), v100, z0, tNozzle, \Wobj := wobjPlatform;
        MoveL Offs(pZeroMute, 0, 0, 50), v100, fine \Inpos := inpos50, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO8_Triple_Retract, 0;

		! Layer: 4
		! LayerParams(infillAngleDegrees=0.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T14
		! M6
		TPWRITE "Layer 4 of 4";
		MoveL Offs(pZeroMute, 0.0, 22.0, 5.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
        WaitTime 0.1;
		MoveL Offs(pZeroMute, -19.053, -11.0, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 19.053, -11.0, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 22.0, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -18.187, -10.5, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 18.187, -10.5, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		MoveL Offs(pZeroMute, 0.0, 21.0, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 0.433, 19.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -0.433, 19.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -1.299, 17.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 1.299, 17.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 2.165, 16.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -2.165, 16.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -3.031, 14.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 3.031, 14.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 3.897, 13.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -3.897, 13.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -4.763, 11.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 4.763, 11.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 5.629, 10.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -5.629, 10.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -6.495, 8.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 6.495, 8.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 7.361, 7.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -7.361, 7.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -8.227, 5.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 8.227, 5.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 9.093, 4.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -9.093, 4.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -9.959, 2.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 9.959, 2.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 10.825, 1.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -10.825, 1.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -11.692, -0.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 11.692, -0.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 12.558, -1.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -12.558, -1.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -13.424, -3.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 13.424, -3.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 14.29, -4.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -14.29, -4.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -15.156, -6.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 15.156, -6.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, 16.022, -7.75, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, -16.022, -7.75, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroMute, -16.888, -9.25, 0.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroMute, 16.888, -9.25, 0.8), v30, z0, tNozzle, \Wobj := wobjPlatform;
		SetDO DO8_Triple_Retract, 1;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;        
        WaitTime 0.15;
		MoveL Offs(pZeroMute, 16.888, -9.25, 5.8), v100, z0, tNozzle, \Wobj := wobjPlatform;
        MoveL Offs(pZeroMute, 0, 0, 50), v100, fine \Inpos := inpos50, tNozzle, \Wobj := wobjPlatform;
		WaitRob \InPos;
		SetDO DO8_Triple_Retract, 0;

		! Extrusion amount for part is (31.6 mm)

		! End Program codes
		SetDO DO1_Auto_Mode, 0;
		SetDO DO5_Program_Feed, 0;
		SetDO DO3_Heat_Bed, 0;
		SetDO DO4_Heat_Nozzle, 0;
		SetDO DO6_Between_Layer_Retract, 0;
	ENDPROC
	PROC Heat()
		SetDO DO4_Heat_Nozzle, 1;
        SetDO DO3_Heat_Bed, 1;
        TPWrite "Caution: Bed and Nozzle heating";
        WaitDI DI3_Nozzle_At_Temp, 1;
        TPWrite "Nozzle is at temperature";
        WaitDI DI2_Bed_At_Temp, 1;
        TPWrite "Bed at temp";
	ENDPROC
ENDMODULE