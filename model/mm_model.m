function C = mm_model(A, B, AW)
% MM_MODEL  Golden model of rtl/accel/systolic_mm.v
%   C = A*B with exact integer arithmetic, then wrapped to a signed AW-bit accumulator
%   (two's-complement wrap-around, identical to the hardware on overflow).
%   A: MxK, B: KxN, signed INT8 (integer-valued doubles). Exact in doubles for AW <= 52.
    C = A * B;
    C = mod(C + 2^(AW-1), 2^AW) - 2^(AW-1);
end
