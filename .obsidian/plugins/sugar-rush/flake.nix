{
  description = "Sugar Rush Plugin For Obsidian";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";

    flake-utils.url = "github:numtide/flake-utils";
    flake-utils.inputs.systems.follows = "systems";

    systems.url = "github:nix-systems/default";
  };

  nixConfig = {
    extra-substituters = ''
      https://cache.nixos.org
      https://nix-community.cachix.org
      https://devenv.cachix.org
    '';
    extra-trusted-public-keys = ''
      cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY=
      nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs=
      devenv.cachix.org-1:w1cLUi8dv3hnoSPGAuibQv+f9TZLr6cv/Hm9XgU50cw=
    '';
    extra-experimental-features = "nix-command flakes";
  };

  outputs = inputs @ {flake-utils, ...}:
    flake-utils.lib.eachSystem [
      "x86_64-linux"
      "i686-linux"
      "x86_64-darwin"
      "aarch64-linux"
      "aarch64-darwin"
    ] (system: let
      pkgs = import inputs.nixpkgs {inherit system;};

      scripts = {
        dx = {
          exec = ''$EDITOR $REPO_ROOT/flake.nix'';
          description = "Edit flake.nix";
        };
        tests = {
          exec = ''go test -v -short ./...'';
          description = "Run short go tests";
        };
        unit-tests = {
          exec = ''go test -v ./...'';
          description = "Run all go tests";
        };
        lint = {
          exec = ''golangci-lint run'';
          description = "Run golangci-lint";
        };
        "generate-api" = {
          exec = ''
            go generate -v "$REPO_ROOT/internal/data/..." &
            proto &
            wait
          '';
          description = "Generate API code";
        };
        format = {
          exec = ''
            export REPO_ROOT=$(git rev-parse --show-toplevel) # needed
            git ls-files \
              --others \
              --exclude-standard \
              --cached \
              -- '*.js' '*.ts' '*.css' '*.md' '*.json' \
              | xargs prettierd --write

          '';
          description = "Format code files across multiple languages";
        };
      };

      # Convert scripts to packages
      scriptPackages =
        pkgs.lib.mapAttrsToList
        (name: script: pkgs.writeShellScriptBin name script.exec)
        scripts;
    in {
      devShells.default = pkgs.mkShell {
        shellHook = ''
          export REPO_ROOT=$(git rev-parse --show-toplevel)
          export CGO_CFLAGS="-O2"
        '';
        packages = with pkgs;
          [
            # Nix
            alejandra
            nixd

            # Node
            nodejs
            nodePackages.prettier
            nodePackages.npm
            bun
            typescript
            typescript-language-server

            # Formatters
            prettierd
          ]
          ++ scriptPackages;
      };
    });
}
