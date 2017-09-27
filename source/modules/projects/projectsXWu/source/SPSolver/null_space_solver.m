%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [~, S_M, V_M] = svd(HC);
% num_pos = sum(sum(S_M > 0.1, []));
% HC_R_M = S_M(1:num_pos, 1:num_pos) * V_M(:, 1:num_pos)';
% ss_HC_R_NS = svd([HC_R_M; NS]);
% num_neg = sum(sum(ss_HC_R_NS < 0.1, []))

QTSC_M = QT' * QT + SC' * SC;
LHS_M = NS * QTSC_M * NS';
if ~isempty(HC)
    HC_NS_M = HC * NS';
    [U_M, S_M, V_M] = svd(HC_NS_M);
    num_pos = sum(sum(S_M > 0.1, []));
    HC_NS_R_M = S_M(1:num_pos, 1:num_pos) * V_M(:, 1:num_pos)';
    HC_NS_Ut_M = U_M(:, 1:num_pos)';
    LHS_M = [LHS_M, HC_NS_R_M'; HC_NS_R_M, zeros(num_pos)];
end

LTSV_M = QT' * LT + SC' * SV;
RHS_M = NS * LTSV_M;
if ~isempty(HV)
    RHS_M = [RHS_M; HC_NS_Ut_M * HV];
end

res_M = LHS_M\RHS_M;
res_M = NS' * res_M;
norm(res_M - result, 'fro')
