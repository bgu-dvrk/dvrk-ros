% call methods to make sure they exist and don't trigger syntax errors
function test_psm_api(arm_name)
    try
        rosnode list;
    catch
        rosinit;
    end
    r = dvrk.psm(arm_name);
    disp('---- Enabling (waiting up to 30s)');
    if ~r.enable(30.0)
        error('Unable to enable arm');
    end
    disp('---- Homing (waiting up to 30s)');
    if ~r.home(30.0)
        error('Unable to home arm');
    end

    % move to 0 position, insert tool using joint command
    joints_home = r.setpoint_js();
    joints_home(:) = 0.0;
    joints_home(3) = 0.10;
    r.wait_while_busy(r.move_jp(joints_home));

    % PSM specific api
    disp('---- Close and open after joint move, jaw should close/open twice');
    r.wait_while_busy(r.insert_jp(0.12));
    r.wait_while_busy(r.jaw.move_jp(deg2rad(0.0)));
    disp(rad2deg(r.jaw.measured_js()));
    r.wait_while_busy(r.jaw.move_jp(deg2rad(60.0)));
    disp(rad2deg(r.jaw.measured_js()));
    r.wait_while_busy(r.jaw.move_jp(deg2rad(0.0)));
    disp(rad2deg(r.jaw.measured_js()));
    r.wait_while_busy(r.jaw.move_jp(deg2rad(60.0)));
    disp(rad2deg(r.jaw.measured_js()));

    % don't forget to cleanup
    disp('---- Delete arm class');
    delete(r);
end
