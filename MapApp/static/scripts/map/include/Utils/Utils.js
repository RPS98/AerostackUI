// List class of [type, fileName]
var blocksClassList = []

class HTMLUtils
{

    static setDict(type, id='none', attributes={}, content) {
        return {
            'type': type,
            'id': id,
            'attributes': attributes,
            'content': content,
        }
    }

    static addTypeDict(type, id='none', attributes={}, ...args) {
        throw new Error('Not implemented');
    }   

    static addTypeHTML(content) {
        throw new Error('Not implemented');
    }

    static _addAttribute(element, id, attributes) {
        if (id != 'none') {
            element.setAttribute('id', id);
        }
        for (let key in attributes) {
            element.setAttribute(key, attributes[key]);
        }
        return element;
    }    

    static addDict(type, id='none', attributes={}, ...args) {
        for (let i=0; i<blocksClassList.length; i++) {
            if (blocksClassList[i][0] == type) {
                return blocksClassList[i][1].addTypeDict(type, id, attributes, ...args);
            }
        }
    }

    static addHTML(parent, childDict) {
        let child = null;
        let flag = true;
        for (let i=0; i<blocksClassList.length; i++) {
            if (blocksClassList[i][0] == childDict.type) {
                child = blocksClassList[i][1].addTypeHTML(childDict.content);
                HTMLUtils._addAttribute(child, childDict.id, childDict.attributes);
                parent.appendChild(child);
                flag = false;
                break; // If types are unique, break after first match to optimize performance
            }
        }
        if (flag) {
            console.log("Unknown type of HTML block");
            console.log(childDict)
            throw new Error('Unknown type of HTML block');
        }
    }

    static addToExistingElement(id, blockList)
    {
        let parent = document.getElementById(id);

        for (let i=0; i<blockList.length; i++) {
            HTMLUtils.addHTML(parent, blockList[i]);
        }
    }

    static initDropDown(id, list, defaultValue) {
        let dropDownBtn = HTMLUtils.addDict('dropDownBtn', `${id}-DropDown-Btn`, {'class': 'btn btn-info'}, defaultValue);
        
        let dropDownExpandList = [];
        for (let i=0; i<list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, {'class': 'btn btn-secondary dropdown-MPMission-item w-75', 'style': "background: #dae8fc"}, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        return HTMLUtils.addDict('dropDown', `${id}-DropDown`, {'class': 'row m-1 gap-2'}, dropDownBtn, dropDownExpand);
    }

    static updateDropDown(id, list) {
        let expand = document.getElementById(`${id}-DropDown-Expand`);
        expand.innerHTML = '';

        let dropDownExpandList = [];
        for (let i=0; i<list.length; i++) {
            dropDownExpandList.push(HTMLUtils.addDict('button', `${id}-DropDown-Expand-${list[i]}`, {'class': 'btn btn-secondary dropdown-MPMission-item w-75', 'style': "background: #dae8fc"}, list[i]));
        }

        let dropDownExpand = HTMLUtils.addDict('dropDownExpand', `${id}-DropDown-Expand`, {}, dropDownExpandList);
        HTMLUtils.addToExistingElement(`${id}-DropDown-menu`, [dropDownExpand]);
    }   
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