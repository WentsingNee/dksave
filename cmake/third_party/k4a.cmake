
#set(K4A_DOWNLOAD_URL "https://download.microsoft.com/download/3/d/6/3d6d9e99-a251-4cf3-8c6a-8e108e960b4b/Azure%20Kinect%20SDK%201.4.1.exe")
#set(K4A_DOWNLOAD_RESULT "Azure Kinect SDK 1.4.1.exe")
#file(DOWNLOAD ${K4A_DOWNLOAD_URL} ${K4A_DOWNLOAD_RESULT})

#find_library(k4a k4a REQUIRED PATHS "C:/Program Files/Azure Kinect SDK v1.4.1/sdk/windows-desktop/amd64/release/lib/")
find_library(libk4a k4a REQUIRED)
target_link_libraries(dksave PUBLIC "${libk4a}")
