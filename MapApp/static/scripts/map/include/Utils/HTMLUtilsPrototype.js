class HTMLUtilsPrototype
{
    static setDict(type, id='none', attributes={}, content) {
        return {
            'type': type,
            'id': id,
            'attributes': attributes,
            'content': content,
        }
    }

    static addDict(type, id='none', attributes={}, ...args) {
        throw new Error('Not implemented');
    }   

    static addHTML(content) {
        throw new Error('Not implemented');
    }
}
