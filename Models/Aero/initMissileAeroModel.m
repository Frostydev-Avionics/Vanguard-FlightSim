function ModelData = initMissileAeroModel()

    % Load the aerodynamic data
    aeroData = readtable('RasAero_MissileCoeffs.csv');

    % Extract unique Mach numbers and AoA values
    MachNumbers = aeroData.Mach;
    AoA = aeroData.Alpha;
    Cd_table = aeroData.CD;
    Cl_table = aeroData.CL;

    unique_MachNumbers = unique(MachNumbers);
    unique_AoA = unique(AoA);

    % Store data in the structure
    ModelData.MachNumbers = unique_MachNumbers;
    ModelData.AoAs = unique_AoA;
    ModelData.Cd_table = Cd_table;
    ModelData.Cl_table = Cl_table;

    % Create Scattered Interpolant
    CD_lookup = scatteredInterpolant(MachNumbers, AoA, Cd_table, 'linear', 'none');
    CL_lookup = scatteredInterpolant(MachNumbers, AoA, Cl_table, 'linear', 'none');

    ModelData.CdLookup = @(inputMach, inputAoA) CD_lookup(inputMach, inputAoA);
    ModelData.ClLookup = @(inputMach, inputAoA) CL_lookup(inputMach, inputAoA);

    % <<< Guesstimated Values >>>

    % Canard Lift Force Deflection Coefficient
    ModelData.canard.CL_delta = 0.2;

    % canard.machNumbers = [0.01, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5];
    % canard.clDelta = [0.000486; 0.000191; 0.000366; 0.000590; 0.000630; 0.000625; 0.004214; 0.000582];

    % Aerodynamic Restoring Moment Coefficients
    ModelData.damping.Cd_x = 0.1;
    ModelData.damping.Cd_y = 0.3;
    ModelData.damping.Cd_z = 0.3;

    % Good estimate is C_n * sideslip angle
    ModelData.Cy_wind = 0.4;
end
