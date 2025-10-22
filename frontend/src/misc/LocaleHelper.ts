import { type InitOptions, type Resource } from "i18next";
import en from "../../res/en.lang.json";
import jp from "../../res/jp.lang.json";
import zh from "../../res/zh.lang.json";

export default class LocaleHelper {
	/**
	 * Flatten a dictionary using WorldEngine's nested config path querying format
	 * For example, the dictionary {a: {b: {c: 1}}} will be flattened to {a/b/c: 1}
	 * @param dict The dictionary to flatten
	 * @returns
	 */
	static getFlatObject(
		dict: Record<string, Record<string, unknown> | unknown>,
	): Record<string, unknown> {
		const flatObject: Record<string, unknown> = {};
		Object.entries(dict).forEach(([key, value]) => {
			if (typeof value === "object" && value !== null) {
				Object.assign(
					flatObject,
					Object.fromEntries(
						Object.entries(
							this.getFlatObject(
								value as Record<
									string,
									Record<string, unknown> | unknown
								>,
							),
						).map(([k, v]) => [key + "/" + k, v]),
					),
				);
			} else {
				flatObject[key] = value;
			}
		});
		return flatObject;
	}

	static getLocaleMapping(): Record<string, Record<string, unknown>> {
		return {
			en: this.getFlatObject(en),
			zh: this.getFlatObject(zh),
			jp: this.getFlatObject(jp),
			ja: this.getFlatObject(jp),
			"zh-CN": this.getFlatObject(zh),
			"zh-TW": this.getFlatObject(zh),
			"zh-HK": this.getFlatObject(zh),
			"zh-SG": this.getFlatObject(zh),
		};
	}

	static getConfig(): InitOptions {
		const resourceConfig: Resource = {};
		Object.entries(this.getLocaleMapping()).forEach(([key, value]) => {
			resourceConfig[key] = {
				translation: value,
			};
		});

		return {
			resources: resourceConfig,
			fallbackLng: "en",
			debug: true,
			detection: {
				order: [
					"navigator",
					"localStorage",
					"sessionStorage",
					"htmlTag",
					"path",
					"subdomain",
				],
				caches: ["localStorage", "sessionStorage"],
				lookupLocalStorage: "i18nextLng",
				lookupSessionStorage: "i18nextLng",
			},
			interpolation: {
				escapeValue: false,
			},
		} as InitOptions;
	}
}
