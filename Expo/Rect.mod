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
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 5.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, 17.75, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -17.25, -17.25, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 17.25, -17.25, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, 17.25, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, -17.25, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -16.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -16.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -15.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -15.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -13.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -13.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -12.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -12.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -10.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -10.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -9.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -9.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -7.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -7.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -6.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -6.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -4.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -4.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -3.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -3.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -1.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -1.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 0.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 0.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 1.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 1.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 3.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 3.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 4.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 4.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 6.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 6.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 7.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 7.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 9.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 9.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 10.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 10.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 12.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 12.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 13.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 13.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 15.0, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 15.0, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 16.5, 0.2), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 16.5, 0.2), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 16.5, 5.2), v100, z0, tNozzle, \Wobj := wobj3Square;

		! Layer: 2
		! LayerParams(infillAngleDegrees=90.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T12
		! M6
		TPWRITE "Layer 2 of 4";
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 5.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, 17.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -17.25, -17.25, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, -17.25, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, 17.25, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -15.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -15.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -13.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -13.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -12.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -12.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -10.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -10.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -9.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -9.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -7.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -7.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -6.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -6.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -4.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -4.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -3.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -3.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -1.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -1.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 0.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 0.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 1.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 1.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 3.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 3.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 4.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 4.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 6.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 6.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 7.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 7.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 9.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 9.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 10.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 10.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 12.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 12.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 13.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 13.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 15.0, -16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 15.0, 16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.5, 16.75, 0.4), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.5, -16.75, 0.4), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.5, -16.75, 5.4), v100, z0, tNozzle, \Wobj := wobj3Square;

		! Layer: 3
		! LayerParams(infillAngleDegrees=0.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T13
		! M6
		TPWRITE "Layer 3 of 4";
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 5.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, 17.75, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -17.25, -17.25, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, -17.25, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, 17.25, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 16.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 16.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 15.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 15.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 13.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 13.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 12.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 12.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 10.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 10.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 9.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 9.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 7.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 7.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 6.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 6.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 4.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 4.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 3.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 3.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, 1.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, 1.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, 0.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, 0.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -1.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -1.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -3.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -3.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -4.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -4.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -6.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -6.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -7.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -7.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -9.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -9.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -10.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -10.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -12.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -12.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -13.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -13.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -15.0, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.75, -15.0, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.75, -16.5, 0.6), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.75, -16.5, 0.6), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.75, -16.5, 5.6), v100, z0, tNozzle, \Wobj := wobj3Square;

		! Layer: 4
		! LayerParams(infillAngleDegrees=90.0, pathWidth=1.5, layerHeight=0.2, infillShiftX=0.0, infillShiftY=0.0, numShells=2, infillOverlap=0.0002)
		! T14
		! M6
		TPWRITE "Layer 4 of 4";
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 5.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 0;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 17.75, 17.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, 17.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.75, -17.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.75, -17.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 17.25, -17.25, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 17.25, 17.25, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, 17.25, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute -17.25, -17.25, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		MoveL Offs(pZeroSquareMute 17.25, -17.25, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 16.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 16.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 15.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 15.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 13.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 13.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 12.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 12.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 10.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 10.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 9.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 9.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 7.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 7.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 6.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 6.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 4.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 4.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 3.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 3.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 1.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 1.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute 0.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute 0.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -1.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -1.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -3.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -3.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -4.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -4.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -6.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -6.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -7.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -7.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -9.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -9.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -10.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -10.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -12.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -12.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -13.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -13.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -15.0, 16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -15.0, -16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.5, -16.75, 0.8), v100, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO5_Program_Feed, 1;
		MoveL Offs(pZeroSquareMute -16.5, 16.75, 0.8), v30, z0, tNozzle, \Wobj := wobj3Square;
		WaitRob \InPos;
		SetDO DO6_Between_Layer_Retract, 1;
		SetDO DO5_Program_Feed, 0;
		MoveL Offs(pZeroSquareMute -16.5, 16.75, 5.8), v100, z0, tNozzle, \Wobj := wobj3Square;

		! Extrusion amount for part is (58.3 mm)

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