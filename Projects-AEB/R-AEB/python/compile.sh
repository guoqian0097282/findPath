set -e

cd "$(dirname "$0")"
export PYTHONPATH="$PWD"

rm -rf build
mkdir -p build/fwdper

python -m nuitka \
  --module fwdper/infer.py \
  --include-package=fwdper \
  --output-dir=build/fwdper \
  --no-pyi-file \
  --remove-output


cp fwdper/__init__.py build/fwdper/
cp fwdper/infer.pyi build/fwdper/
cp -r fwdper/assets build/fwdper/assets
cp -r test_data build/fwdper/test_data

python -m build --wheel --outdir . --no-isolation