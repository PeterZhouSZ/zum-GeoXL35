%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QT = [6, 2, 1; 2, 5, 2; 1, 2, 4];
% LT = [8; 3; 3];
% HC = [1, 0, 1; 0, 1, 1];
% % HV = [3; 0];
% HV = [0; 0];
% SC = [0, 100, 0];
% SV = [100];

% QTSC = QT;
% QTSV = LT;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if isempty(SC)
%     QTSC = QT' * QT ;
%     QTSV = QT' * LT;
% else
%     QTSC = QT' * QT + SC' * SC;
%     QTSV = QT' * LT + SC' * SV;
% end

if exist('HC_NS', 'var')
    HC = HC_NS;
    HV = HV_NS;
end
QTSCHC = [QTSC, HC'; HC, zeros(size(HC, 1))];
LTSVHV = [QTSV; HV];
res_M = QTSCHC \ LTSVHV;
res_M = res_M(1:size(QTSC, 1));
norm(res_M - res, 'fro')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[U_M, S_M, V_M] = svd(HC);
num_pos = sum(sum(S_M > 0.1, []));
HC_R_M = S_M(1:num_pos, 1:num_pos) * V_M(:, 1:num_pos)';
HC_Ut_M = U_M(:, 1:num_pos)';
HC_N_M = V_M(:, num_pos+1:end)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% null-space solver
if isempty(HC)
    LHS_M = QTSC;
    RHS_M = QTSV;
else
    HC_R_y_M = HC_R_M \ (HC_Ut_M * HV);
    HC_N_QTSC_y_M = HC_N_M * (QTSC * HC_R_y_M);
    LHS_M = HC_N_M * QTSC * HC_N_M';
    
    RHS_M = HC_N_M * QTSV - HC_N_QTSC_y_M;
end

z = LHS_M \ RHS_M;
if isempty(HC)
    res_M = z;
else
    res_M = HC_N_M' * z + HC_R_y_M;
end
norm(res_M - res, 'fro')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% direct solver
% rr = diag(S) * V_T(1:num_pos,:); % note non-unique of singular vector
LHS_M = [QTSC, HC_R_M'; HC_R_M, zeros(size(HC_R_M, 1))];

HC_Ut_HV_M = HC_Ut_M * HV;
RHS_M = [QTSV; HC_Ut_HV_M];

res_M = LHS_M \ RHS_M;
res_M = res_M(1:size(QTSC, 1));
norm(res_M - res, 'fro')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% direct null-space
if isempty(HC)
    N_G_N_M = QTSC;
    N_c = QTSV;
else
    N_G_N_M = HC_N_M * QTSC * HC_N_M';
    N_c = HC_N_M * QTSV;
end
z = N_G_N_M \ N_c;
if isempty(HC)
    res_M = z;
else
    res_M = HC_N_M' * z;
end
norm(res_M - res, 'fro')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HC_R_NS_M = [HC_R; NS];
% min(svd(HC_R_NS_M));
