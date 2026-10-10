double hFirstLayer = 0.01;
int wingn = 21 ; // value used by bash shell should be constant number
int tipn = 13 ; // value used by bash shell should be constant number
double tipoffset = hFirstLayer;
double spanlength = 5.;
double domainz = spanlength + 4.;

// Source rounded flat plate in InputMesh (dimensions are absolute lengths).
double sourceChord = 1.0;
double foilAngleDegrees = 15.0;
double leadingEdgeRadius = 0.025;
double trailingEdgeRadius = 0.025;

// Keep the source trailing edge fixed; linearly taper the chord along z.
double rootChord = 1.3;
double tipChord = 0.7;

// Blend the fluid mesh to the unchanged far field in the source (u,v) frame.
double taperNearNormal = 0.1;
double taperFarNormal = 5.0;
double taperNearUpstream = -1.0;
double taperFarUpstream = -5.0;
