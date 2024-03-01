# Mapping: Hardware to IO number

The following table gives an overview of the mapping of the hardware number to the IO variable numbers, which can be used in the software programming.

You will find the corresponding Excel-Table via the following [**_IRAS Students/General/01-IRAS_Wiki/2-Projects/2.1-Automated_Item_Picking/10_KUKA_WoV/IO Mapping.xlsx_**](https://hskarlsruhede.sharepoint.com/:x:/s/Robolab/EckyS9jFJg1JsJkgt-PI7WIBAil-Je8FX_XW09xrqo0VMg?e=toe4zT)

## Inputs

| Master Channel  | Connected Slave Name  | Slave Channel | Name                                          | KR C4 Name                                 | Data Type | Field Module Adress  |
|-----------------|-----------------------|---------------|-----------------------------------------------|--------------------------------------------|-----------|----------------------|
| X01             | AL2401                | X1.0          | Reedkontakt 1: Zylinder 1 ausgefahren         | $IN[132]                                   | BOOL      | 1072                 |
| X01             | AL2401                | X1.1          | Reedkontakt 1: Zylinder 3 ausgefahren         | $IN[133]                                   | BOOL      | 1073                 |
| X01             | AL2401                | X1.2          | Reedkontakt 2: Zylinder 1 eingefahren         | $IN[134]                                   | BOOL      | 1074                 |
| X01             | AL2401                | X1.3          | Reedkontakt 2: Zylinder 3 eingefahren         | $IN[135]                                   | BOOL      | 1075                 |
| X01             | AL2401                | X1.4          | Reedkontakt 1: Zylinder 2 ausgefahren         | $IN[136]                                   | BOOL      | 1076                 |
| X01             | AL2401                | X1.5          | Reedkontakt 1: Zylinder 4 ausgefahren         | $IN[137]                                   | BOOL      | 1077                 |
| X01             | AL2401                | X1.6          | Reedkontakt 2: Zylinder 2 eingefahren         | $IN[138]                                   | BOOL      | 1078                 |
| X01             | AL2401                | X1.7          | Reedkontakt 2: Zylinder 4 eingefahren         | $IN[139]                                   | BOOL      | 1079                 |
| X05             | AL1332                |               | UD-Sensor 01 / Messwert Teil 1                | $IN[25]#G                                  | USINT     | 1232                 |
| X05             | AL1332                |               | UD-Sensor 01 / Messwert Teil 2                | $IN[33]#G                                  | USINT     | 1242                 |
| X05             | AL1332                |               | UD-Sensor 01 / SP1                            | $IN[41]                                    | BOOL      | 1240                 |
| X05             | AL1332                |               | UD-Sensor 01 / SP2                            | $IN[42]                                    | BOOL      | 1241                 |
| X06             | AL1332                |               | UD-Sensor 02 / Messwert Teil 1                | $IN[43]#G                                  | USINT     | 1264                 |
| X06             | AL1332                |               | UD-Sensor 02 / Messwert Teil 2                | $IN[51]#G                                  | USINT     | 1274                 |
| X06             | AL1332                |               | UD-Sensor 02 / SP1                            | $IN[59]                                    | BOOL      | 1272                 |
| X06             | AL1332                |               | UD-Sensor 02 / SP2                            | $IN[60]                                    | BOOL      | 1273                 |
| X07             | AL1332                |               | UD-Sensor 03 / Messwert Teil 1                | $IN[61]#G                                  | USINT     | 1296                 |
| X07             | AL1332                |               | UD-Sensor 03 / Messwert Teil 2                | $IN[69]#G                                  | USINT     | 1306                 |
| X07             | AL1332                |               | UD-Sensor 03 / SP1                            | $IN[77]                                    | BOOL      | 1304                 |
| X07             | AL1332                |               | UD-Sensor 03 / SP2                            | $IN[78]                                    | BOOL      | 1305                 |
| X08             | AL1332                |               | UD-Sensor 04 / Messwert Teil 1                | $IN[79]#G                                  | USINT     | 1328                 |
| X08             | AL1332                |               | UD-Sensor 04 / Messwert Teil 2                | $IN[87]#G                                  | USINT     | 1338                 |
| X08             | AL1332                |               | UD-Sensor 04 / SP1                            | $IN[95]                                    | BOOL      | 1336                 |
| X08             | AL1332                |               | UD-Sensor 04 / SP2                            | $IN[96]                                    | BOOL      | 1337                 |

## Outputs

| Master Channel  | Connected Slave Name  | Slave Channel | Name                | KR C4 Name          | Data Type | Field Module Adress  |
|-----------------|-----------------------|---------------|---------------------|---------------------|-----------|----------------------|
| X02             | AL2330                | X1.2          | Ejektor 1           | $OUT[33]            | BOOL      | 12769                |
| X02             | AL2330                | X1.3          | Ejektor 2           | $OUT[34]            | BOOL      | 12761                |
| X02             | AL2330                | X1.4          | Ejektor 3           | $OUT[35]            | BOOL      | 12770                |
| X02             | AL2330                | X1.5          | Ejektor 4           | $OUT[36]            | BOOL      | 12762                |
| X03             | AL2330                | X1.4          | Adidas Zylinder 1   | $OUT[37]            | BOOL      | 12826                |
| X03             | AL2330                | X1.5          | Adidas Zylinder 2   | $OUT[38]            | BOOL      | 12834                |
| X04             | MPA-L                 | X1.0          | Zylinder 1 einfahren| $OUT[39]            | BOOL      | 12888                |
| X04             | MPA-L                 | X1.1          | Zylinder 1 ausfahren| $OUT[40]            | BOOL      | 12889                |
| X04             | MPA-L                 | X1.2          | Zylinder 2 einfahren| $OUT[41]            | BOOL      | 12890                |
| X04             | MPA-L                 | X1.3          | Zylinder 2 ausfahren| $OUT[42]            | BOOL      | 12891                |
| X04             | MPA-L                 | X1.4          | Zylinder 3 einfahren| $OUT[43]            | BOOL      | 12892                |
| X04             | MPA-L                 | X1.5          | Zylinder 3 ausfahren| $OUT[44]            | BOOL      | 12893                |
| X04             | MPA-L                 | X1.6          | Zylinder 4 einfahren| $OUT[45]            | BOOL      | 12894                |
| X04             | MPA-L                 | X1.7          | Zylinder 4 ausfahren| $OUT[46]            | BOOL      | 12895                |
