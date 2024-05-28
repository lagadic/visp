{
  description = "Open Source Visual Servoing Platform";

  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs =
    { flake-utils, nixpkgs, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        visp = pkgs.callPackage ./default.nix { };
      in
      {
        packages.default = visp;
      }
    );
}
