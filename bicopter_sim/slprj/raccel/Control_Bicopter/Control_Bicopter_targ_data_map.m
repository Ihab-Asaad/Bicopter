  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtP)
    ;%
      section.nData     = 62;
      section.data(62)  = dumData; %prealloc
      
	  ;% rtP.D_Phi
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.D_Psi
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.D_Theta
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtP.D_Z
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtP.I_Phi
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtP.I_Psi
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtP.I_Theta
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtP.I_Z
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtP.P_Phi
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtP.P_Psi
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtP.P_Theta
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtP.P_Z
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtP.Z_step
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtP.copter_mass
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtP.g
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtP.time_delay
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtP.PIDPhi_InitialConditionForFilter
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtP.PIDZ_InitialConditionForFilter
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtP.PIDTheta_InitialConditionForFilter
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtP.PIDPsi_InitialConditionForFilter
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtP.PIDPhi_InitialConditionForIntegrator
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtP.PIDZ_InitialConditionForIntegrator
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtP.PIDTheta_InitialConditionForIntegrator
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 22;
	
	  ;% rtP.PIDPsi_InitialConditionForIntegrator
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 23;
	
	  ;% rtP.PIDPhi_N
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 24;
	
	  ;% rtP.PIDZ_N
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 25;
	
	  ;% rtP.PIDTheta_N
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 26;
	
	  ;% rtP.PIDPsi_N
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 27;
	
	  ;% rtP.Constant_Value
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 28;
	
	  ;% rtP.Constant_Value_opjr4pzkxp
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 29;
	
	  ;% rtP.TransportDelay3_InitOutput
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 30;
	
	  ;% rtP.TransportDelay1_InitOutput
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 31;
	
	  ;% rtP.TransportDelay2_InitOutput
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 32;
	
	  ;% rtP.TransportDelay4_InitOutput
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 33;
	
	  ;% rtP.Gain_Gain
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 34;
	
	  ;% rtP.Saturation_F1_UpperSat
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 35;
	
	  ;% rtP.Saturation_F1_LowerSat
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 36;
	
	  ;% rtP.Saturation_F2_UpperSat
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 37;
	
	  ;% rtP.Saturation_F2_LowerSat
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 38;
	
	  ;% rtP.Saturation_F3_UpperSat
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 39;
	
	  ;% rtP.Saturation_F3_LowerSat
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 40;
	
	  ;% rtP.Saturation_F4_UpperSat
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 41;
	
	  ;% rtP.Saturation_F4_LowerSat
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 42;
	
	  ;% rtP.Gain1_Gain
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 43;
	
	  ;% rtP.Gain2_Gain
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 44;
	
	  ;% rtP.Integrator_IC
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 45;
	
	  ;% rtP.Integrator_IC_gptrc5suwu
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 46;
	
	  ;% rtP.speedgain_Gain
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 47;
	
	  ;% rtP.const_Gain
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 48;
	
	  ;% rtP.Gain_Gain_nt13gxv4cf
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 49;
	
	  ;% rtP.speedgain_Gain_cayz03alro
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 50;
	
	  ;% rtP.const_Gain_nyshjvfom1
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 51;
	
	  ;% rtP.Gain_Gain_nz2osdhexb
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 52;
	
	  ;% rtP.Gain_Gain_m3wdc4uyp3
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 53;
	
	  ;% rtP.Gain_Gain_crytncxhkk
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 54;
	
	  ;% rtP.Phi_ref_Value
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 55;
	
	  ;% rtP.Psi_ref_Value
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 56;
	
	  ;% rtP.Theta_ref_Value
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 57;
	
	  ;% rtP.Constant2_Value
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 58;
	
	  ;% rtP.Constant2_Value_bijw4dwdaf
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 59;
	
	  ;% rtP.Constant_Value_pr21eedhgl
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 60;
	
	  ;% rtP.Constant1_Value
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 61;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtB)
    ;%
      section.nData     = 31;
      section.data(31)  = dumData; %prealloc
      
	  ;% rtB.alnu044j5k
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.g41m4pmy1w
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.kuc2djevrk
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.pzxksc1hoi
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtB.m1dkcmbbey
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 6;
	
	  ;% rtB.onyodjbxop
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 7;
	
	  ;% rtB.ibl1bvoot5
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 8;
	
	  ;% rtB.ozxva0egn1
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 9;
	
	  ;% rtB.eslkbwnlvg
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 10;
	
	  ;% rtB.e04tk5zbbe
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 11;
	
	  ;% rtB.lgntheub31
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 12;
	
	  ;% rtB.ms5qo0q32e
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 13;
	
	  ;% rtB.agzdgrfhap
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 14;
	
	  ;% rtB.eczwzi2yjw
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 15;
	
	  ;% rtB.k4jt1nr2ej
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 19;
	
	  ;% rtB.gudikbezkv
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 20;
	
	  ;% rtB.b2gk3wukry
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 24;
	
	  ;% rtB.nxogfkpljw
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 25;
	
	  ;% rtB.gipodijd1l
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 29;
	
	  ;% rtB.e4vcd3rbgn
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 30;
	
	  ;% rtB.igfxglvoam
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 34;
	
	  ;% rtB.oqquwhnyjp
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 95;
	
	  ;% rtB.p1zfkdy0ss
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 104;
	
	  ;% rtB.omd4dozqfc
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 105;
	
	  ;% rtB.kdhjnpyryt
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 108;
	
	  ;% rtB.aj3d0xadxg
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 109;
	
	  ;% rtB.hl1tydtlp3
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 110;
	
	  ;% rtB.je2xqylh2t
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 111;
	
	  ;% rtB.jcianyi4kp
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 112;
	
	  ;% rtB.o2kcntnybn
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 113;
	
	  ;% rtB.hqhvl0otti
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 117;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 5;
    sectIdxOffset = 1;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtDW)
    ;%
      section.nData     = 16;
      section.data(16)  = dumData; %prealloc
      
	  ;% rtDW.ecgt4jrfbm
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.fkv2s4cq4q
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.pd2cnxedgx
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.jm54tlayqp
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.ikaqdpe14g
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.hquutdkqak
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.ccksc230fn
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.kbkgyu20nl
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.b2lbbxidhp
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.fvqjeh3ony
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 10;
	
	  ;% rtDW.ggqhsa10cs
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 12;
	
	  ;% rtDW.kaia4u11f2
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 13;
	
	  ;% rtDW.k21rn4i5xu.modelTStart
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 14;
	
	  ;% rtDW.mdms1xibhu.modelTStart
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 15;
	
	  ;% rtDW.bfxyvq4vri.modelTStart
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 16;
	
	  ;% rtDW.aqigcjflxp.modelTStart
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 17;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 19;
      section.data(19)  = dumData; %prealloc
      
	  ;% rtDW.g2lkbxgjhp.TUbufferPtrs
	  section.data(1).logicalSrcIdx = 16;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.lizyfiedvc.TUbufferPtrs
	  section.data(2).logicalSrcIdx = 17;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtDW.mx22ld2hzj.TUbufferPtrs
	  section.data(3).logicalSrcIdx = 18;
	  section.data(3).dtTransOffset = 4;
	
	  ;% rtDW.diva5n50pk.TUbufferPtrs
	  section.data(4).logicalSrcIdx = 19;
	  section.data(4).dtTransOffset = 6;
	
	  ;% rtDW.nomdnwo2sd.LoggedData
	  section.data(5).logicalSrcIdx = 20;
	  section.data(5).dtTransOffset = 12;
	
	  ;% rtDW.ouuolo4dzo.LoggedData
	  section.data(6).logicalSrcIdx = 21;
	  section.data(6).dtTransOffset = 13;
	
	  ;% rtDW.nlope1xt2k.LoggedData
	  section.data(7).logicalSrcIdx = 22;
	  section.data(7).dtTransOffset = 14;
	
	  ;% rtDW.nwoe35ohwt.LoggedData
	  section.data(8).logicalSrcIdx = 23;
	  section.data(8).dtTransOffset = 15;
	
	  ;% rtDW.edvzy5545y
	  section.data(9).logicalSrcIdx = 24;
	  section.data(9).dtTransOffset = 16;
	
	  ;% rtDW.m2zz55qyqo
	  section.data(10).logicalSrcIdx = 25;
	  section.data(10).dtTransOffset = 17;
	
	  ;% rtDW.nbicecjjvm
	  section.data(11).logicalSrcIdx = 26;
	  section.data(11).dtTransOffset = 18;
	
	  ;% rtDW.pntjz4vsvn
	  section.data(12).logicalSrcIdx = 27;
	  section.data(12).dtTransOffset = 19;
	
	  ;% rtDW.ghee1gdvk4
	  section.data(13).logicalSrcIdx = 28;
	  section.data(13).dtTransOffset = 20;
	
	  ;% rtDW.avtvu4a0bu
	  section.data(14).logicalSrcIdx = 29;
	  section.data(14).dtTransOffset = 21;
	
	  ;% rtDW.b1ivukxskp
	  section.data(15).logicalSrcIdx = 30;
	  section.data(15).dtTransOffset = 22;
	
	  ;% rtDW.hzasxj2szj
	  section.data(16).logicalSrcIdx = 31;
	  section.data(16).dtTransOffset = 23;
	
	  ;% rtDW.f0bb2ffysq
	  section.data(17).logicalSrcIdx = 32;
	  section.data(17).dtTransOffset = 24;
	
	  ;% rtDW.fm533wjgzd
	  section.data(18).logicalSrcIdx = 33;
	  section.data(18).dtTransOffset = 25;
	
	  ;% rtDW.ihz200qec4.LoggedData
	  section.data(19).logicalSrcIdx = 34;
	  section.data(19).dtTransOffset = 26;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% rtDW.j03obu1eim.Tail
	  section.data(1).logicalSrcIdx = 35;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.cadeui54l4.Tail
	  section.data(2).logicalSrcIdx = 36;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.dbj1dpcqc0.Tail
	  section.data(3).logicalSrcIdx = 37;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.ko54z1uphk.Tail
	  section.data(4).logicalSrcIdx = 38;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.j24ttftmn0
	  section.data(5).logicalSrcIdx = 39;
	  section.data(5).dtTransOffset = 16;
	
	  ;% rtDW.ckkmlyrmch
	  section.data(6).logicalSrcIdx = 40;
	  section.data(6).dtTransOffset = 17;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% rtDW.g5ujhhy5gx
	  section.data(1).logicalSrcIdx = 41;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.eghku2cvih
	  section.data(2).logicalSrcIdx = 42;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.gte2z2pt0y
	  section.data(3).logicalSrcIdx = 43;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.n4vwxzlvdz
	  section.data(4).logicalSrcIdx = 44;
	  section.data(4).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtDW.ldjeeunpd1
	  section.data(1).logicalSrcIdx = 45;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.iura2nrvwp
	  section.data(2).logicalSrcIdx = 46;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 2369118705;
  targMap.checksum1 = 162466512;
  targMap.checksum2 = 3416941678;
  targMap.checksum3 = 3742909486;

