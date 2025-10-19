#!/usr/bin/bash

# format is always "Author/Model" for example "Qwen/Qwen3-0.6B"
authormodel="$1"

# quantization
quantization="$2"

cd /data

# and the exact download link, for example "https://huggingface.co/Qwen/Qwen3-0.6B"
gitclone="https://huggingface.co/${authormodel}"

# exact folder we're going to store the original model in, for example "Qwen3-0.6B"
modeldir="${authormodel#*/}"

# where we're going to stash the compiled version of the model, for example "qwen3-0.6b-q4f16_ft-MLC"
mlcdir="${modeldir,,}-${quantization}-MLC"

echo "Author/Model: ${authormodel}"
echo "Quantization: ${quantization}"
echo "Model Dir:    ${modeldir}"
echo "MLC Dir:      ${mlcdir}"


git clone "${gitclone}" 

rm -rf "${mlcdir}"

mlc_llm convert_weight "${modeldir}" --quantization "${quantization}" -o "${mlcdir}" --device cuda

mlc_llm gen_config "${modeldir}" --quantization "${quantization}" --max-batch-size 1 --context-window-size 16536 --conv-template chatml -o "${mlcdir}"

mlc_llm compile "${mlcdir}" -o "${mlcdir}/lib.so" --device cuda

echo "Done!  You can now launch the model using: "
echo "mlc_llm chat /data/${mlcdir} --model-lib /data/${mlcdir}/lib.so --device cuda"

# mlc_llm chat /data/${mlcdir} --model-lib /data/${mlcdir}/lib.so --device cuda

