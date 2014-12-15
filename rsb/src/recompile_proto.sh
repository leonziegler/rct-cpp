#!/bin/bash
	
BASEDIR=$(dirname $(readlink -f $0))
if [ ! -n "$prefix" ]; then
	echo "Environment variable \"\$prefix\" must be set!"
	exit 1
fi
cd ${BASEDIR}
echo "Re-compiling:"
echo "    FrameTransform.proto"
echo ""
protoc \
--proto_path=rct/proto:$prefix/share/rst0.11/proto/stable/ \
--cpp_out=rct/impl \
rct/proto/FrameTransform.proto \
echo "done."
