some infos:

have set some ros parameters before you start the diagnostic engine.

e.g.
	rosparam load launch/SimSys_analyzer_load.yaml /rosha_diagnostic_aggregator
	to load the analyzer parameters for the RobotTestSystem. Check what is commented!
	
	or use the -loadFile paramters of the diagnostic_aggregator to load the paramterers
