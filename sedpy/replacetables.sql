DELETE FROM cable;
DELETE FROM component;
DELETE FROM congender;
DELETE FROM connector;
DELETE FROM jack;
DELETE FROM line;
DELETE FROM pin;

LOAD DATA INFILE '/data/sedweb/out/cable' INTO TABLE cable;
LOAD DATA INFILE '/data/sedweb/out/component' INTO TABLE component;
LOAD DATA INFILE '/data/sedweb/out/congender' INTO TABLE congender;
LOAD DATA INFILE '/data/sedweb/out/connector' INTO TABLE connector;
LOAD DATA INFILE '/data/sedweb/out/jack' INTO TABLE jack;
LOAD DATA INFILE '/data/sedweb/out/line' INTO TABLE line;
LOAD DATA INFILE '/data/sedweb/out/pin' INTO TABLE pin;
