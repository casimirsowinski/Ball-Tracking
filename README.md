# Ball-Tracking
Experimental Object Tracking Program Designed for a computer vision class at Portland State University

For this project, I used:
  - OpenCV 2.4.13
    - http://opencv.org/downloads.html
  - Intel(R) RealSense(TM) SDK
    - https://software.intel.com/en-us/intel-realsense-sdk
  - Visual Studio Community 2015
    - https://www.visualstudio.com/downloads/
    
If you want or need to build a VS project and simply use these .cpp files then you'll need to make the take these steps:
  - Make a new, project
    - Choose Visual C++ from Templates
    - Choose Empty Project 
    - Name it what you want
  - Right-click on project name in Solution Explorer window, select properties
  - Add these fields to the entries shown here:
  
    - Properties				
      - VC++ Directories			
        - Include Directories		
          - C:\opencv\build\x86\vc12\lib;C:\opencv\build\include	
          - C:\Program Files (x86)\Intel\RSSDK\lib\Win32;C:\Program Files (x86)\Intel\RSSDK\include	
        - Library Directories		
          - C:\opencv\build\x86\vc12\lib;C:\opencv\build\include	
          - C:\Program Files (x86)\Intel\RSSDK\lib\Win32;C:\Program Files (x86)\Intel\RSSDK\include	
      - Linker			
        - General		
          - Additional Library Directories	
            - C:\opencv\build\x86\vc12\lib;C:\opencv\build\include
            - C:\Program Files (x86)\Intel\RSSDK\lib\Win32;C:\Program Files (x86)\Intel\RSSDK\include
          - Use Library Dependency Inputs	
            - YES
        - Input		
          - Additional Depenencies	
            - libpxc.lib;libpxc_d.lib;libpxcmd.lib;libpxcmd_d.lib
            - opencv_calib3d2413d.lib;opencv_contrib2413d.lib;opencv_core2413d.lib;opencv_features2d2413d.lib;opencv_flann2413d.lib;opencv_gpu2413d.lib;opencv_highgui2413d.lib;opencv_imgproc2413d.lib;opencv_legacy2413d.lib;opencv_ml2413d.lib;opencv_nonfree2413d.lib;opencv_objdetect2413d.lib;opencv_ocl2413d.lib;opencv_photo2413d.lib;opencv_stitching2413d.lib;opencv_superres2413d.lib;opencv_ts2413d.lib;opencv_video2413d.lib;opencv_videostab2413d.lib
            
  

  
