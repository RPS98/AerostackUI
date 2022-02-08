// TODO: Separate in classes

class HTMLDicts
{
    static addDict(type, content, id='none', attributes={}) {
        return {
            'type': type,
            'content': content,
            'id': id,
            'attributes': attributes,
            
        }
    }

    static addExistingElement(id, content, attributes={}) {
        return HTMLDicts.addDict(
            'existing',
            content,
            id,
            attributes
        );
    }

    static addCollapse(name, content, id, attributes={}) {
        return HTMLDicts.addDict(
            'collapse',
            {
                'name': name,
                'content': content
            },
            id,
            attributes
        );
    }

    static addRow(content, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'row',
            content,
            id,
            attributes
        );
    }

    static addInput(type, defaultValue, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'input',
            {
                'inputType': type,
                'placeholder': defaultValue,
            },
            id,
            attributes
        );
    }

    static addButton(text, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'button',
            text,
            id,
            attributes
        );
    }

    static addLabel(text, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'label',
            text,
            id,
            attributes
        );
    }

    static addDiv(content, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'div',
            content,
            id,
            attributes
        );
    }

    static addRowLines(content, id='none', attributes={}) {
        return HTMLDicts.addDict(
            'rowLines',
            content,
            id,
            attributes
        );
    }
}

class HTMLBlocks
{

    static _addAttribute(element, id, attributes) {
        if (id != 'none') {
            element.setAttribute('id', id);
        }
        for (let key in attributes) {
            element.setAttribute(key, attributes[key]);
        }
        return element;
    }

    static recursiveAddDicts(parent, htmlInfo) {

        let id = htmlInfo.id;
        let type = htmlInfo.type;
        let attributes = htmlInfo.attributes;
        let content = htmlInfo.content;

        let child = null;
        
        switch (type) {
            case 'collapse':
                child = HTMLBlocks.addCollapseGroup(id, content.name, content.content);
                break;
            case 'row':
                child = HTMLBlocks.addRow(content);
                break;
            case 'input':
                child = HTMLBlocks.addInput(content);
                break;
            case 'button':
                child = HTMLBlocks.addButton(content);
                break;
            case 'label':
                child = HTMLBlocks.addLabel(content);
                break;
            case 'div':
                child = HTMLBlocks.addDiv(content);
                break;
            case 'rowLines':
                child = HTMLBlocks.addRowLines(content);
                break;
            case 'existing':
                throw new Error('Can not exits existing element inside other existing element');
                break;
            default:
                throw new Error(`Unknown type: ${type}`);
                break;
        }

        HTMLBlocks._addAttribute(child, id, attributes);

        parent.appendChild(child)
    }

    static addDict(htmlInfo)
    {
        let id = htmlInfo.id;
        let type = htmlInfo.type;
        let content = htmlInfo.content;

        if (type != 'existing') {
            throw new Error('First one must be existing');
        }

        let parent = document.getElementById(id);

        for (let i=0; i<content.length; i++) {
            HTMLBlocks.recursiveAddDicts(parent, content[i]);
        }
    }

    static addCollapseGroup(groupId, groupName, groupContent) {

        let div = document.createElement('div');
        let btn = document.createElement('button');
        let div_collapse = document.createElement('div');

        btn.setAttribute('class', 'btn btn-secondary btn-collap-toggle');
        btn.setAttribute('type', 'button');
        btn.setAttribute('data-bs-toggle', 'collapse');
        btn.setAttribute('data-bs-target', '#' + groupId);
        btn.innerHTML = `${groupName} <i class="icon-collap-toggle fas fa-plus"></i>`;

        div_collapse.setAttribute('class', 'collapse multi-collapse');
        div_collapse.setAttribute('id', groupId);
        
        for (let i=0; i<groupContent.length; i++) {
            HTMLBlocks.recursiveAddDicts(div_collapse, groupContent[i]);
        }

        div.setAttribute('class', 'd-grid gap-2 m-2');
        div.appendChild(btn);
        div.appendChild(div_collapse);

        return div;
    }

    static addRow(input = []) {
        let len = input.length;
        let div = document.createElement('div');
        div.setAttribute('class', 'row my-1 mx-1');

        let col = 12 / len;

        for (let i = 0; i < len; i++) {
            let div_col = document.createElement('div');
            div_col.setAttribute('class', `col-md-${col}`);
            HTMLBlocks.recursiveAddDicts(div_col, input[i]);
            div.appendChild(div_col);
        }

        return div;
    }
    
    static addLabel(content) {
        let label = document.createElement('label');
        label.innerHTML = content;
        return label;
    }

    static addInput(content) {
        let input = document.createElement('input');
        input.setAttribute('type', content.inputType);
        input.setAttribute('placeholder', content.placeholder);
        return input;
    }

    static addButton(content) {
        let button = document.createElement('button');
        button.setAttribute('type', 'submit');
        button.innerHTML = content;
        return button;
    }

    static addDiv(content) {
        let div = document.createElement('div');
        for (let i = 0; i < content.length; i++) {
            HTMLBlocks.recursiveAddDicts(div, content[i]);
        }
        return div;
    }

    // static addOneInputParam(name, defaultValue, paramType, paramId) {
    //     let label = HTMLUtils.addLabel(name);
    //     let input = HTMLUtils.addInput(paramType, paramId, defaultValue);
    //     HTMLUtils.addRow([label, input]);
    // }
}

class Utils 
{
     /**
      * Dumb function to initialize callbacks
      * @param {Object} input - functions inputs 
      */
    static _nullFunct(input) {
        console.log("Dumb callback function");
    };
 
     /**
      * Add a callback function to a toggle button
      * @param {string}   button_id  - id of the button
      * @param {function} callback   - callback when press button
      * @param {list}     args       - arguments list pass to callback
      */
    static addButtonCallback(button_id, callback=Utils._nullFunct, args=[]) {
        // get button element
        const btn = document.getElementById(button_id);

        // add an event listener to the button
        btn.addEventListener('click', (e) => {
            // disable the refresh on the page when submit
            e.preventDefault(); 

            // call the button callback
            callback(args);
        });
    }
 
     /**
      * Add a listener for number inputs by a button
      * @param {string}   button_id - id of the button
      * @param {string}   inputs_id - list of form inputs id
      * @param {string}   names_id  - name of elements in the list
      * @param {function} callback  - callback when press button
      * @param {list}     args      - arguments list pass to callback
      */ 
    static addFormCallback(button_id, inputs_id, names_id, callback=Utils._nullFunct, args=[]) {

        // get button element
        const btn = document.getElementById(button_id);

        // check if inputs and names has the same length
        if (inputs_id.length != names_id.length) {
            throw "Input id and names id lengths are not equals";
        }

        // add a button listener
        btn.addEventListener('click', (e) => {
            // disable the refresh on the page when submit
            e.preventDefault();

            // for each input elemenet, get the value
            let inputs = [];
            for (let i = 0; i < inputs_id.length; i++) {
                inputs[names_id[i]] = parseFloat(document.getElementById(inputs_id[i]).value);
            }

            // call the callback
            callback(args, inputs);
        });
    }

    static setColor(map, UAV_color) 
    {
        map.pm.setPathOptions({color: UAV_color});
    }

    static onlyUnique(value, index, self) {
        return self.indexOf(value) === index;
    }
      
 }