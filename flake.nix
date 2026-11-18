{
  description = "Open Source Visual Servoing Platform";

  inputs = {
    nixpkgs.url = "https://channels.nixos.org/nixpkgs-unstable/nixexprs.tar.zst";
  };

  outputs = inputs: {
    packages = builtins.mapAttrs (system: pkgs: {
      visp = pkgs.callPackage ./default.nix { };

      default = inputs.self.packages.${system}.visp;
    }) inputs.nixpkgs.legacyPackages;
  };
}
