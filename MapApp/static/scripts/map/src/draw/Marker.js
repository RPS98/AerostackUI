class Marker extends DrawManager
{
    constructor(test, codeDrawOptions={}, userDrawOptions={}) {
        super('Marker', codeDrawOptions, userDrawOptions);

        this.codeDrawOptions = codeDrawOptions;
        this.userDrawOptions = userDrawOptions;

        this.test_var = test;
    }

    codeDraw(values, missionId, uavList, height, options={}) {
        this.codeDrawOptions['missionId'] = missionId;
        this.codeDrawOptions['uavList'] = uavList;
        this.codeDrawOptions['height'] = height;
        return super.codeDraw(values, Object.assign({}, this.codeDrawOptions, options));
    }

    userDraw(missionId, uavList, height, options={}) {
        this.codeDrawOptions['missionId'] = missionId;
        this.codeDrawOptions['uavList'] = uavList;
        this.codeDrawOptions['height'] = height;
        return super.userDraw(Object.assign({}, this.userDrawOptions, options));
    }

    showInfo() {
        throw new Error("Method not implemented.");
    }

    sendInfo() {
        throw new Error("Method not implemented.");
    }
}