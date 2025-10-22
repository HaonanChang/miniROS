export default class StringHelper {
	public static getCapitalizedString(str: string) {
		str = str.toLowerCase();

		return str.charAt(0).toUpperCase() + str.slice(1);
	}

	public static getPrettyJson(json: Record<string, unknown>) {
		const sortedKeys = Object.keys(json).sort((a, b) => {
			const aIsObjectOrArray =
				typeof json[a] === "object" && json[a] !== null;
			const bIsObjectOrArray =
				typeof json[b] === "object" && json[b] !== null;
			if (aIsObjectOrArray && !bIsObjectOrArray) return 1;
			if (!aIsObjectOrArray && bIsObjectOrArray) return -1;
			return 0;
		});

		const sortedJson = sortedKeys.reduce(
			(acc: Record<string, unknown>, key) => {
				acc[key] = json[key];
				return acc;
			},
			{},
		);

		return JSON.stringify(sortedJson, null, 2);
	}
}
