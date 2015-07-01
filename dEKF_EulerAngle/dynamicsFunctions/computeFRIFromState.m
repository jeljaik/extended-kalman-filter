function [x,y] = computeFRIFromState(XUpt,P,tK,idx)

    muo_x_B = XUpt(idx:end,10); muo_x_B_sigma = squeeze(2*sqrt(P(10,10,idx:end)))';
    muo_y_B = XUpt(idx:end,11); muo_y_B_sigma = squeeze(2*sqrt(P(11,11,idx:end)))';
    muo_z_B = XUpt(idx:end,12); muo_z_B_sigma = squeeze(2*sqrt(P(12,12,idx:end)))';

    muc_x_B = XUpt(idx:end,16); muc_x_B_sigma = squeeze(2*sqrt(P(16,16,idx:end)))';
    muc_y_B = XUpt(idx:end,17); muc_y_B_sigma = squeeze(2*sqrt(P(17,17,idx:end)))';
    muc_z_B = XUpt(idx:end,18); muc_z_B_sigma = squeeze(2*sqrt(P(18,18,idx:end)))';

    b_p_foot =[0 0 0.18102]';
    b_p_eqn = b_p_foot;
    foot_R_b = [ 0  0 -1;
                 0  1  0;
                 1  0  0];
    foot_R_eqn = [0 0 1;
                  0 -1 0;
                  1 0 0];
    eqn_R_foot = foot_R_eqn';         
    %% because of assumptions of foot reference frame in the eqn
    eqn_R_b = eqn_R_foot * foot_R_b;
    foot_adjT_b = [foot_R_b zeros(3) ; -foot_R_b*S(b_p_foot) eye(3) ];

    eqn_adjT_b = [eqn_R_b zeros(3);-eqn_R_b*S(b_p_foot) eye(3)];

    %SK = S(); %transforming body frame to foot frame
    %* foDash' + muoDash';

    fc_x_B = XUpt(idx:end,13); fc_x_B_sigma = squeeze(2*sqrt(P(13,13,idx:end)))';
    fc_y_B = XUpt(idx:end,14); fc_y_B_sigma = squeeze(2*sqrt(P(13,13,idx:end)))';
    fc_z_B = XUpt(idx:end,15); fc_z_sigma = squeeze(2*sqrt(P(13,13,idx:end)))';

    fo_x_B = XUpt(idx:end,7); fo_x_B_sigma = squeeze(2*sqrt(P(7,7,idx:end)))';
    fo_y_B= XUpt(idx:end,8); fo_y_B_sigma = squeeze(2*sqrt(P(8,8,idx:end)))';
    fo_z_B = XUpt(idx:end,9); fo_z_B_sigma = squeeze(2*sqrt(P(9,9,idx:end)))';


    mu_o_B = [muo_x_B muo_y_B muo_z_B];
    mu_o_B_sigma = [muo_x_B_sigma;muo_y_B_sigma;muo_z_B_sigma];

    mu_c_B = [muc_x_B muc_y_B muc_z_B];
    %mu_c_B_sigma = [muc_x_B_sigma;muc_y_B_sigma;muc_z_B_sigma]:

    f_o_B = [fo_x_B fo_y_B fo_z_B];
    f_o_B_sigma = [fo_x_B_sigma;fo_y_B_sigma;fo_z_B_sigma];

    f_c_B = [fc_x_B fc_y_B fc_z_B];
    %f_c_B_sigma = [fc_x_B_sigma;fc_y_B_sigma;fc_z_B_sigma];
    
    %muo_x_sigma(i),muo_y_sigma(i),fo_z_B_sigma(i))

    %mu_o = -SK*f_o_B' + mu_o_B';
    %mu_c = -SK*f_c_B' + mu_c_B';
    %[fo_muo_eqn] = eqn_adjT_b * [f_o_B mu_o_B]';
    [fo_muo_eqn] = foot_adjT_b * [f_o_B mu_o_B]';
    %[fc_muc_eqn] = eqn_adjT_b * [f_c_B mu_c_B]';

   % [fo_muo_eqn_sigma] = (foot_adjT_b * [f_o_B_sigma;mu_o_B_sigma]')' * foot_adjT_b';
    
    f_o = fo_muo_eqn(1:3,:);
    mu_o = fo_muo_eqn(4:6,:);

    %f_c = fc_muc_eqn(1:3,:);
    %mu_c = fc_muc_eqn(4:6,:);

    muo_x = mu_o(2,:)';
    muo_y = mu_o(3,:)';

    %muc_x = mu_c(1,:)';
    %muc_y = mu_c(2,:)';

    %fo_z = fo_z_B;
    %fc_z = fc_z_B;
    fo_z = f_o(1,:);
    %fc_z = f_c(3,:);

    %muo = SK*fo_z' + muo_x_B;
    %muc_x = SK*fo_z' + muo_x_B;

    %pcop_expect = zeros(size(muo_x,1),2);pcop_covariance = zeros(size(muo_y,1),2,2);
    pfri_expect = zeros(size(muo_x,1),2);
    pfri_expect_eqn = pfri_expect;
    pfri_covariance = zeros(size(muo_y,1),2,2);
    w = zeros(size(muo_x,1));
    h = zeros(size(muo_x,1));

    for i = 1:size(muo_x,1)
        %[pcop_expect(i,:),pcop_covariance(i,:,:)] = computeCOP(muc_x(i),muc_y(i),fc_z(i),...
        %    muc_x_sigma(i),muc_y_sigma(i),fc_z_sigma(i)) ;
        [pfri_expect_eqn(i,1:2),pfri_covariance_eqn(i,:,:)] = computeFRI(muo_x(i),muo_y(i),fo_z(i),...
            muo_x_B_sigma(i),muo_y_B_sigma(i),fo_z_B_sigma(i));
        w(i) = squeeze(2*sqrt(pfri_covariance(i,1,1)));
        h(i) = squeeze(2*sqrt(pfri_covariance(i,2,2)));
    end
    %feetDistanceFromCenter = 0.07;%0.07; %40 cm
    pfri_expect(1:end,1:2) = pfri_expect_eqn(:,1:2);
    %(foot_R_eqn(2:3,1:2)*pfri_expect_eqn(:,1:2)')';
    x = pfri_expect(:,1);
    y = pfri_expect(:,2);
end