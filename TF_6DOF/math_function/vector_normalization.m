function normalized_vec = vector_normalization(vec)
    % VECTOR_NORMALIZATION Normalize a vector to unit length
    %
    % Input:
    %   vec            - Input vector (any dimension)
    %
    % Output:
    %   normalized_vec - Unit vector in same direction
    %
    % Note: No check for zero vector - caller must ensure vec is non-zero
    
    normalized_vec = vec / norm(vec);
end