{ pkgs ? import <nixpkgs> {}
, board ? "omnibusf4pro"
, target ? "plane"
, dev ? false
}: with pkgs;

stdenv.mkDerivation {
  name = "ardupilot";

  # Don't copy source directory to store if we are running in nix-shell
  src = if lib.inNixShell then null else lib.cleanSourceWith {
    filter = name: type: let baseName = baseNameOf (toString name); in !(
      (baseName == "build" && type == "directory")
    );
    src = ./.;
  };

  nativeBuildInputs = [
    which
    gawk
    gcc-arm-embedded
    git
    (python3.withPackages (p: with p; [
      future
      pyserial
    ] ++ lib.optionals dev [
      # mavproxy joystick support
      pygame
      pyyaml
    ]))
  ] ++ lib.optionals dev [
    glibcLocales
    astyle
    procps
    ccls
    mavproxy
  ];

  postPatch = ''
    patchShebangs waf
  '';

  # Prevent waf from using the host compiler
  preConfigure = ''
    unset CXX CC OBJCOPY SIZE
  '';

  configurePhase = ''
    runHook preConfigure
    ./waf configure --board '${board}'
    runHook postConfigure
  '';

  buildPhase = ''
    runHook preBuild
    ./waf '${target}'
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
