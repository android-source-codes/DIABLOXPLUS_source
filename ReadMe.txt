1. cd /Your_SourceCode
  for example :
	cd /alps

2. export PATH=/Your_Toolchain_PATH/:$PATH
   
   for example :
	export PATH=/alps/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin:/alps/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin:/alps/prebuilts/misc/linux-x86/make:$PATH

3. Build Command:
   ./makeMtk -t diabloxplus_kk n k

