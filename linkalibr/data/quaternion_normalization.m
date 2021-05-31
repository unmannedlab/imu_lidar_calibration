function quat_normalized = quaternion_normalization(quat)
    if(quat(4) < 0)
        quat = -quat;
    end
    quat_normalized = quat./norm(quat);
end
