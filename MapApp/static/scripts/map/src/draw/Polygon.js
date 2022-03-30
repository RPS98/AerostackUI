class Polygon extends DrawManager
{
    constructor(status, name, options = undefined, layerOptions = undefined) {
        super(status, 'Polygon', name, options, layerOptions);
    }

    codeDraw(values, options = undefined, layerOptions = {}, missionId = undefined) {
        if (values.length < 1) {
            return;
        }
        if (missionId !== undefined) {
            layerOptions['color'] = M.MISSION_MANAGER.getColors(missionId)[1];
            this.layerOptions['color'] = layerOptions['color'];
        }
        return super.codeDraw(values, options, layerOptions);
    }

    _addChangeCallback(id, info) {
        let htmlId = [];
        let nameId = [];
        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            htmlId.push(`${id}-${i}-lat`);
            htmlId.push(`${id}-${i}-lng`);

            nameId.push(`${i}-lat`);
            nameId.push(`${i}-lng`);
        }
        Utils.addFormCallback(`${id}-change`, htmlId, nameId, this._changeCallback.bind(this), id, info);
    }

    _changeCallback(myargs, inputs) {
        let layer = myargs[1].layer;
        
        let values = [];
        let len = Math.floor(Object.keys(inputs).length / 2);
        for (let i = 0; i < len; i++) {
            values.push([]);
        }
        for (var key in inputs) {
            let value = inputs[key];
            let keyParse = key.split('-');
            let index1 = parseInt(keyParse[0]);
            if (keyParse[1] == 'lat') {
                values[index1][0] = value;
            } else if (keyParse[1] == 'lng') {
                values[index1][1] = value;
            }
        }

        layer.setLatLngs(values);
    }

    drawInfoAdd(htmlId, info, name = "Line", initialHtml = [], endHtml = undefined, uavPickerType = undefined) {

        let id = htmlId + '-' + info.id;

        let values = info.layer._latlngs[0];
        for (let i = 0; i < values.length; i++) {
            let lat = values[i].lat;
            let lng = values[i].lng;

            let latDict = HTMLUtils.addDict('input', `${id}-${i}-lat`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lat, 6) }, 'number', 'Latitude');
            let lngDict = HTMLUtils.addDict('input', `${id}-${i}-lng`, { 'class': 'form-control', 'required': 'required', 'value': Utils.round(lng, 6) }, 'number', 'Longitude');
            let row = HTMLUtils.addDict('splitDivs', 'none', { 'class': 'row my-1 mx-1' }, [latDict, lngDict], { 'class': 'col-6' });

            initialHtml.push(row);
        }

        return super.drawInfoAdd(htmlId, info, name, initialHtml, endHtml, uavPickerType);
    }
}

class Area extends Polygon
{
    constructor(status, options = {}, layerOptions = undefined) {

        // Default values
        let algorithmList = ['Back and force', 'Spiral'];
        let algorithm = 'Back and force';
        let streetSpacing = 1;
        let wpSpace = 1;
        let verticalOverlap = 50;
        let horizontalOverlap = 50;

        if (options.algorithmList == undefined) {
            options.algorithmList = algorithmList;
        }
        if (options.algorithm == undefined) {
            options.algorithm = algorithm;
        }
        if (options.streetSpacing == undefined) {
            options.streetSpacing = streetSpacing;
        }
        if (options.wpSpace == undefined) {
            options.wpSpace = wpSpace;
        }
        if (options.verticalOverlap == undefined) {
            options.verticalOverlap = verticalOverlap;
        }
        if (options.horizontalOverlap == undefined) {
            options.horizontalOverlap = horizontalOverlap;
        }

        super(status, 'Area', options, layerOptions);
    }

    drawInfoAdd(htmlId, info) {
        let name = 'Area';
        let endHtml = [];
        let id = htmlId + '-' + info.id;

        console.log(this)
        let options = this.options.drawManager.options;

        endHtml.push(HTMLUtils.initDropDown(`${id }-Swarming`, options.algorithmList, options.algorithm));

        let streetSpacing = HTMLUtils.addDict('input', `${id}-streetSpacing`, { 'class': 'form-control', 'required': 'required', 'value': options.streetSpacing}, 'number', 'Value');
        let streetSpacingBtn = HTMLUtils.addDict('button', `${id}-streetSpacingBtn`, { 'class': 'btn btn-primary' }, 'Set street space (m)');
        let streetSpacingDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [streetSpacing]);
        let streetSpacingBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [streetSpacingBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [streetSpacingDiv, streetSpacingBtnDiv]));

        let wpSpace = HTMLUtils.addDict('input', `${id}-wpSpace`, { 'class': 'form-control', 'required': 'required', 'value': options.wpSpace}, 'number', 'Value');
        let wpSpaceBtn = HTMLUtils.addDict('button', `${id}-wpSpaceBtn`, { 'class': 'btn btn-primary' }, 'Set wp space (m)');
        let wpSpaceDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [wpSpace]);
        let wpSpaceBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [wpSpaceBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [wpSpaceDiv, wpSpaceBtnDiv]));

        let verticalOverlap = HTMLUtils.addDict('input', `${id}-verticalOverlap`, { 'class': 'form-control', 'required': 'required', 'value': options.verticalOverlap}, 'number', 'Value');
        let verticalOverlapBtn = HTMLUtils.addDict('button', `${id}-verticalOverlapBtn`, { 'class': 'btn btn-primary' }, 'Set vertical overlap (%)');
        let verticalOverlapDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [verticalOverlap]);
        let verticalOverlapBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [verticalOverlapBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [verticalOverlapDiv, verticalOverlapBtnDiv]));

        let horizontalOverlap = HTMLUtils.addDict('input', `${id}-horizontalOverlap`, { 'class': 'form-control', 'required': 'required', 'value': options.horizontalOverlap}, 'number', 'Value');
        let horizontalOverlapBtn = HTMLUtils.addDict('button', `${id}-horizontalOverlapBtn`, { 'class': 'btn btn-primary' }, 'Set horizontal overlap (%)');
        let horizontalOverlapDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col' }, [horizontalOverlap]);
        let horizontalOverlapBtnDiv = HTMLUtils.addDict('div', `none`, { 'class': 'col-6' }, [horizontalOverlapBtn]);
        endHtml.push(HTMLUtils.addDict('div', `none`, { 'class': 'row my-1 mx-1' }, [horizontalOverlapDiv, horizontalOverlapBtnDiv]));
        

        let swarmPlannerCollapse = HTMLUtils.addDict('collapse', `${this.htmlId}-SwarmCollapse`, {}, 'Planner', true, endHtml);

        return super.drawInfoAdd(htmlId, info, name, undefined, swarmPlannerCollapse, 'checkbox');
    }

    drawInfoInitialize(id, info) {
        Utils.addButtonsCallback(`${id}-Swarming-item`, this.clickAlgorithmsListCallback.bind(this), id, info);
        Utils.addFormCallback(`${id}-streetSpacingBtn`, [`${id}-streetSpacing`], ['streetSpacingValue'], this.streetSpacingCallback.bind(this), id, info);
        Utils.addFormCallback(`${id}-wpSpaceBtn`, [`${id}-wpSpace`], ['wpSpaceValue'], this.wpSpaceCallback.bind(this), id, info);

        super.drawInfoInitialize(id, info);
    }

    clickAlgorithmsListCallback(e, args) {
        args[1].drawManager.options.algorithm = e.innerText;

        let button = document.getElementById(`${args[0]}-Swarming-DropDown-Btn`);
        button.innerHTML = e.innerHTML;
    }

    streetSpacingCallback(myargs, inputs) {
        myargs[1].drawManager.options.streetSpacing = inputs.streetSpacingValue;
    }

    wpSpaceCallback(myargs, inputs) {
        myargs[1].drawManager.options.wpSpace = inputs.wpSpaceValue;
    }
}

