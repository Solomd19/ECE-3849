# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\supad\workspace_v10\ece3849d21_lab2_dmsolomon/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\supad\workspace_v10\ece3849d21_lab2_dmsolomon/src/makefile.libs clean

