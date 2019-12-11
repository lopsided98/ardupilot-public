{ pkgs ? import <nixpkgs> {}, board ? "MatekH743", dev ? false }:
with pkgs;

stdenv.mkDerivation {
  name = "ardupilot";

  # Don't copy source directory to store if we are running in nix-shell
  src = if lib.inNixShell then null else lib.cleanSourceWith {
    filter = name: type: let baseName = baseNameOf (toString name); in !(
      (baseName == "build" && type == "directory")
    );
    src = ./.;
  };

  nativeBuildInputs = [ which gawk gcc-arm-embedded git ] ++
    (with python3Packages; [ future pyserial empy pexpect setuptools ]) ++
    lib.optional dev [ glibcLocales astyle mavproxy procps ccls ];

  postPatch = ''
    patchShebangs waf
  '';

  # Prevent waf from using the host compiler
  preConfigure = ''
    unset CXX CC OBJCOPY SIZE
  '';

  # fortify hardening enables -O2, which causes extra warnings in the
  # AP_Scripting binding generator, and upstream has no interest in fixing
  # them.
  hardeningDisable = [ "fortify" ];

  configurePhase = ''
    runHook preConfigure
    ./waf configure --board '${board}'
    runHook postConfigure
  '';

  buildPhase = ''
    runHook preBuild
    ./waf plane
    runHook postBuild
  '';

  installPhase = ''
    runHook preInstall
    mv 'build/${board}/bin' "$out"
    runHook postInstall
  '';

  shellHook = ''
    runHook preConfigure
  '';
}
