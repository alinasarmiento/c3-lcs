# -*- mode: python -*-
# vi: set ft=python :

workspace(name = "c3-lcs")

# dairlib can use either a local version of drake or a pegged revision
# If the environment variable LOCAL_DRAKE_PATH is set, it will use
# a local version, ad the specified path. Otherwise, it will get a pegged
# revision from github.
# As an example,
#  export LOCAL_DRAKE_PATH=/home/user/workspace/drake

# Choose a revision of Drake to use.
DRAKE_COMMIT = "v1.28.0"

DRAKE_CHECKSUM = "6ff298d7fbc33cb17963509f86fcd9cb6816d455b97b3fd589e1085e0548c2fe"
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
#DRAKE_CHECKSUM = "0" * 64

# Load an environment variable.
load("//:environ.bzl", "drake_repository")
load("//:environ.bzl", "inekf_repository")

drake_repository(name="drake_path")
inekf_repository(name="inekf_path")
load("@drake_path//:environ.bzl", "LOCAL_DRAKE_PATH")
load("@inekf_path//:environ.bzl", "LOCAL_INEKF_PATH")

# The WORKSPACE file does not permit `if` statements, so we handle the local
# option by toying with the repository names.  The selected repository is named
# "@drake", the other is named "@drake_ignored".
(_http_drake_repo_name, _local_drake_repo_name) = (
    "drake_ignored" if LOCAL_DRAKE_PATH else "drake",
    "drake" if LOCAL_DRAKE_PATH else "drake_ignored",
)

# Maybe download Drake.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = _http_drake_repo_name,
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT.strip("v")),
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
)

# Maybe use a local checkout of Drake.
print("Using LOCAL_DRAKE_PATH={}".format(LOCAL_DRAKE_PATH)) if LOCAL_DRAKE_PATH else None  # noqa

local_repository(
    name = _local_drake_repo_name,
    path = LOCAL_DRAKE_PATH,
)

# Reference external software libraries and tools per Drake's defaults.  Some
# software will come from the host system (Ubuntu or macOS); other software
# will be downloaded in source or binary form from github or other sites.
load("@drake//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@c3-lcs//tools/workspace/osqp:repository.bzl", "osqp_repository")

osqp_repository(name = "osqp")

load("@c3-lcs//tools/workspace/drake_models:repository.bzl", "drake_models_repository")

drake_models_repository(name = "drake_models")

# dairlib can use either a local version of invariant-ekf or a pegged revision
# If the environment variable LOCAL_INEKF_PATH is set, it will use
# a local version, ad the specified path. Otherwise, it will get a pegged
# revision from github.
# As an example,
#  export LOCAL_INEKF_PATH=/home/user/workspace/invariant-ekf

# Choose a revision of InEKF to use.
INEKF_COMMIT = "7fde9f84dbe536ba9439a3b8c319efb51ff760dd"

INEKF_CHECKSUM = "f87e3262b0c9c9237881fcd539acd1c60000f97dfdfa47b0ae53cb7a0f3256e4"

# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# INEKF_CHECKSUM = "0" * 64

# The WORKSPACE file does not permit `if` statements, so we handle the local
# option by toying with the repository names.  The selected repository is named
# "@inekf", the other is named "@inekf_ignored".
(_http_inekf_repo_name, _local_inekf_repo_name) = (
    "inekf_ignored" if LOCAL_INEKF_PATH else "inekf",
    "inekf" if LOCAL_INEKF_PATH else "inekf_ignored",
)

# Maybe download InEKF.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = _http_inekf_repo_name,
    sha256 = INEKF_CHECKSUM,
    strip_prefix = "invariant-ekf-{}".format(INEKF_COMMIT),
    urls = [x.format(INEKF_COMMIT) for x in [
        "https://github.com/DAIRLab/invariant-ekf/archive/{}.tar.gz",
    ]],
)

# Maybe use a local checkout of InEKF.
print("Using LOCAL_INEKF_PATH={}".format(LOCAL_INEKF_PATH)) if LOCAL_INEKF_PATH else None  # noqa

local_repository(
    name = _local_inekf_repo_name,
    path = LOCAL_INEKF_PATH,
)

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# buildifier is written in Go and hence needs rules_go to be built.
# See https://github.com/bazelbuild/rules_go for the up to date setup instructions.
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "d6b2513456fe2229811da7eb67a444be7785f5323c6708b38d851d2b51e54d83",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.30.0/rules_go-v0.30.0.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.30.0/rules_go-v0.30.0.zip",
    ],
)

load("@io_bazel_rules_go//go:deps.bzl", "go_rules_dependencies")

go_rules_dependencies()

load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains")

go_register_toolchains(version = "1.17.2")

http_archive(
    name = "bazel_gazelle",
    sha256 = "de69a09dc70417580aabf20a28619bb3ef60d038470c7cf8442fafcf627c21cb",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.24.0/bazel-gazelle-v0.24.0.tar.gz",
    ],
)

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")

# If you use WORKSPACE.bazel, use the following line instead of the bare gazelle_dependencies():
# gazelle_dependencies(go_repository_default_config = "@//:WORKSPACE.bazel")
gazelle_dependencies()

http_archive(
    name = "com_google_protobuf",
    sha256 = "3bd7828aa5af4b13b99c191e8b1e884ebfa9ad371b0ce264605d347f135d2568",
    strip_prefix = "protobuf-3.19.4",
    urls = [
        "https://github.com/protocolbuffers/protobuf/archive/v3.19.4.tar.gz",
    ],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

protobuf_deps()

http_archive(
    name = "com_github_bazelbuild_buildtools",
    sha256 = "ae34c344514e08c23e90da0e2d6cb700fcd28e80c02e23e4d5715dddcb42f7b3",
    strip_prefix = "buildtools-4.2.2",
    urls = [
        "https://github.com/bazelbuild/buildtools/archive/refs/tags/4.2.2.tar.gz",
    ],
)
