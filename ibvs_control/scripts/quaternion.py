from tf.transformations import quaternion_inverse, quaternion_multiply

def quaternion(p):
    assert len(p)==3
    return [p[0], p[1], p[2], 0.0]

def transform_by_quaternion(q_b2a, p_a, q_a2b):
    q_a = quaternion(p_a)
    return quaternion_multiply(q_b2a, quaternion_multiply(q_a, q_a2b))[:3]